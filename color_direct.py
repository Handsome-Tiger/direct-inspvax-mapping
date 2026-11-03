"""Color direct INS geometry using approximate historical camera calibration."""
import argparse,csv,json
from collections import deque
from pathlib import Path
import cv2,numpy as np,rosbag
from scipy.spatial.transform import Rotation,Slerp
from map_direct import ROT,parse

K=np.array([[849.6656839939023,0,692.8241643540154],[0,761.6655369255013,550.33647885436],[0,0,1]],float)
D=np.array([-.05729528706141188,.1210407244166642,.001274128378760289,.0004389741530109464])
RCL=np.array([[.00438814,-.999807,-.0191582],[-.00978695,.0191145,-.999769],[.999942,.00457463,-.00970118]])
TCL=np.array([.016069,.0871753,-.0718021])
def writer(path,valid_field):
 f=path.open('xb+');f.write(b'ply\nformat binary_little_endian 1.0\nelement vertex ');offset=f.tell();f.write(b'0'*20);f.write(b'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n'+(b'property uchar color_valid\n' if valid_field else b'')+b'end_header\n');return f,offset
def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--ins',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--camera-offset',type=float,default=-.026);p.add_argument('--max-frames',type=int);p.add_argument('--calibration',type=Path,help='JSON camera calibration; omit for legacy provisional values');a=p.parse_args()
 global K,D,RCL,TCL
 if a.calibration:
  calibration=json.loads(a.calibration.read_text(encoding='utf-8'));K=np.asarray(calibration['intrinsics'],float);D=np.asarray(calibration['distortion'],float);RCL=np.asarray(calibration['rotation_camera_lidar'],float);TCL=np.asarray(calibration['translation_camera_lidar_m'],float)
  if K.shape!=(3,3) or RCL.shape!=(3,3) or TCL.shape!=(3,) or not np.all(np.isfinite(K)) or not np.allclose(RCL.T@RCL,np.eye(3),atol=1e-4) or np.linalg.det(RCL)<.99:raise ValueError('Invalid camera calibration')
 with a.ins.open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
 t=np.array([315964800+int(float(r['gps_week']))*604800+float(r['gps_seconds'])-18 for r in rows]);pos=np.array([[float(r[k]) for k in ('east_m','north_m','up_m')] for r in rows]);q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]);status=np.array([int(float(r['ins_status'])) for r in rows]);invalid=(np.linalg.norm(np.diff(pos,axis=0),axis=1)>3*np.diff(t))|(status[:-1]!=3)|(status[1:]!=3);slerp=Slerp(t,Rotation.from_quat(q))
 a.output.mkdir(parents=True,exist_ok=True);complete=a.output/'cloud_direct_ins_color_complete.ply';strict=a.output/'cloud_direct_ins_color_strict.ply'
 if complete.exists() or strict.exists():raise FileExistsError('Refusing to overwrite color outputs')
 dtype=np.dtype([('xyz','<f4',3),('rgb','u1',3),('valid','u1')]);sdtype=np.dtype([('xyz','<f4',3),('rgb','u1',3)])
 n=colored=frames=paired=0;deltas=[];window=deque()
 with rosbag.Bag(str(a.bag)) as cb,rosbag.Bag(str(a.bag)) as lb:
  ci=iter(cb.read_messages(topics=['/left_camera/image/compressed']));next_cam=None
  def readcam():
   try:
    _,m,_=next(ci);ct=m.header.stamp.to_sec()+a.camera_offset;return (ct,m)
   except StopIteration:return None
  next_cam=readcam();cf,co=writer(complete,True);sf,so=writer(strict,False)
  try:
   for _,raw,_ in lb.read_messages(topics=['/livox/lidar'],raw=True):
    if a.max_frames is not None and frames>=a.max_frames:break
    base,pts=parse(raw[1]);frames+=1;xyz=np.column_stack([pts['x'],pts['y'],pts['z']]);valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>=.8)&((pts['tag']&0x30)!=0x30);xyz=xyz[valid];offset=pts['offset'][valid];inten=pts['reflectivity'][valid];times=base*1e-9+offset.astype(float)*1e-9;inside=(times>=t[0])&(times<=t[-1]);idx=np.clip(np.searchsorted(t,times,side='right')-1,0,len(t)-2);good=inside&~invalid[idx];xyz=xyz[good];times=times[good];inten=inten[good];idx=idx[good]
    if not len(xyz):continue
    u=(times-t[idx])/(t[idx+1]-t[idx]);world=slerp(times).apply((ROT@xyz.T).T)+pos[idx]*(1-u[:,None])+pos[idx+1]*u[:,None];ref=float(np.max(times))
    while next_cam is not None and next_cam[0]<=ref+.12:window.append(next_cam);next_cam=readcam()
    while window and window[0][0]<ref-.12:window.popleft()
    candidates=sorted(window,key=lambda x:abs(x[0]-ref))[:3];sums=np.zeros((len(world),3),dtype=np.float32);counts=np.zeros(len(world),dtype=np.uint8)
    if candidates:paired+=1;deltas.append(abs(candidates[0][0]-ref))
    for ct,m in candidates:
     if ct<t[0] or ct>t[-1]:continue
     j=np.clip(np.searchsorted(t,ct,side='right')-1,0,len(t)-2)
     if invalid[j]:continue
     image=cv2.imdecode(np.frombuffer(m.data,dtype=np.uint8),cv2.IMREAD_COLOR)
     if image is None:continue
     alpha=(ct-t[j])/(t[j+1]-t[j]);pc=pos[j]*(1-alpha)+pos[j+1]*alpha;body=slerp([ct]).inv().apply(world-pc);lidar=(ROT.T@body.T).T;camera=(RCL@lidar.T).T+TCL;front=camera[:,2]>.2
     ii=np.flatnonzero(front)
     if not len(ii):continue
     uv,_=cv2.projectPoints(camera[ii],np.zeros(3),np.zeros(3),K,D);uv=uv.reshape(-1,2);pix=np.rint(uv).astype(np.int32);h,w=image.shape[:2];ok=(pix[:,0]>=0)&(pix[:,0]<w)&(pix[:,1]>=0)&(pix[:,1]<h);ii=ii[ok];pix=pix[ok]
     if not len(ii):continue
     flat=pix[:,1]*w+pix[:,0];depth=camera[ii,2];zbuf=np.full(h*w,np.inf,dtype=np.float32);np.minimum.at(zbuf,flat,depth);vis=depth<=zbuf[flat]+.3;ii=ii[vis];pix=pix[vis];sums[ii]+=image[pix[:,1],pix[:,0],::-1];counts[ii]+=1
    has=counts>0;rgb=np.repeat(inten[:,None],3,axis=1);rgb[has]=np.clip(np.rint(sums[has]/counts[has,None]),0,255).astype(np.uint8);r=np.empty(len(world),dtype=dtype);r['xyz']=world;r['rgb']=rgb;r['valid']=has;cf.write(r.tobytes());s=np.empty(int(has.sum()),dtype=sdtype);s['xyz']=world[has];s['rgb']=rgb[has];sf.write(s.tobytes());n+=len(world);colored+=int(has.sum())
    if frames%100==0:print('frames',frames,'points',n,'true_color',colored,'paired',paired,flush=True)
   for f,o,count in [(cf,co,n),(sf,so,colored)]:f.seek(o);f.write(f'{count:020d}'.encode('ascii'))
  finally:cf.close();sf.close()
 report={'frames_read':frames,'complete_points':n,'true_color_points':colored,'true_color_fraction':colored/n if n else 0,'paired_frames':paired,'mean_nearest_camera_difference_s':float(np.mean(deltas)) if deltas else None,'camera_time_offset_s':a.camera_offset,'camera_calibration':str(a.calibration) if a.calibration else 'legacy scaled provisional calibration','intrinsics':K.tolist(),'distortion':D.tolist(),'rotation_camera_lidar':RCL.tolist(),'translation_camera_lidar_m':TCL.tolist(),'method':'direct device INS poses; nearest three camera frames, z-buffer projection','warnings':['camera intrinsics/extrinsics and transform direction require independent validation','camera exposure timestamps not recorded; offset estimated from frame cadence, not calibrated exposure time','INS/LiDAR rotation candidate fitted on same data, translation assumed zero','1 Hz attitude interpolation, no high-frequency scan deskew','color coverage is not an accuracy measure']};(a.output/'color_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()

