"""Project Livox returns using device INSPVAX; no filter or pose graph."""
import argparse,csv,json,struct
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import rosbag

ROT=np.array([[.004904127434223249,.011983900039529028,.9999161643227654],[.025502069678610486,.9996014748230979,-.012105204401195183],[-.9996627401159934,.025559297160328877,.004596558762140435]])
DTYPE=np.dtype([('offset','<u4'),('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('tag','u1'),('line','u1')])
def parse(data):
    frame_len=struct.unpack_from('<I',data,12)[0];off=16+frame_len
    base,count,lidar,*_=struct.unpack_from('<QIB3B',data,off);off+=16
    length=struct.unpack_from('<I',data,off)[0];off+=4
    n=min(count,length,(len(data)-off)//DTYPE.itemsize)
    return base,np.frombuffer(data,dtype=DTYPE,count=n,offset=off)
def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--ins',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--max-frames',type=int);p.add_argument('--gps-utc-leap',type=float,default=18.);p.add_argument('--max-position-step',type=float,default=3.);a=p.parse_args()
    with a.ins.open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
    t=np.array([315964800+int(float(r['gps_week']))*604800+float(r['gps_seconds'])-a.gps_utc_leap for r in rows]);pos=np.array([[float(r[k]) for k in ('east_m','north_m','up_m')] for r in rows]);q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]);status=np.array([int(float(r['ins_status'])) for r in rows]);
    if len(t)<2 or np.any(np.diff(t)<=0):raise ValueError('INS timestamps must increase')
    dt=np.diff(t);jump=np.linalg.norm(np.diff(pos,axis=0),axis=1);invalid=(jump>a.max_position_step*dt)|(status[:-1]!=3)|(status[1:]!=3)
    a.output.mkdir(parents=True,exist_ok=True);out=a.output/'cloud_direct_ins_geometry.ply'
    if out.exists():raise FileExistsError(out)
    data_type=np.dtype([('xyz','<f4',3),('intensity','u1')]);written=frames=0;skipped=0;skipped_outside=0
    slerp=Slerp(t,Rotation.from_quat(q));
    with out.open('xb+') as out_f,rosbag.Bag(str(a.bag)) as bag:
        out_f.write(b'ply\nformat binary_little_endian 1.0\nelement vertex ');count_pos=out_f.tell();out_f.write(b'0'*20);out_f.write(b'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar intensity\nend_header\n')
        for _,raw,_ in bag.read_messages(topics=['/livox/lidar'],raw=True):
            if a.max_frames is not None and frames>=a.max_frames:break
            base,pts=parse(raw[1]);frames+=1
            xyz=np.column_stack([pts['x'],pts['y'],pts['z']]);r=np.linalg.norm(xyz,axis=1);valid=np.isfinite(xyz).all(axis=1)&(r>=.8)&(pts['tag']&0x30!=0x30)
            xyz=xyz[valid];offset=pts['offset'][valid];intensity=pts['reflectivity'][valid]
            times=base*1e-9+offset.astype(float)*1e-9;inside=(times>=t[0])&(times<=t[-1]);skipped_outside+=int((~inside).sum());idx=np.clip(np.searchsorted(t,times,side='right')-1,0,len(t)-2);good=inside&~invalid[idx];skipped+=int((inside&~good).sum())
            if not np.any(good):continue
            xyz=xyz[good];times=times[good];intensity=intensity[good];idx=idx[good];u=(times-t[idx])/(t[idx+1]-t[idx]);p0=pos[idx]*(1-u[:,None])+pos[idx+1]*u[:,None];world=slerp(times).apply((ROT@xyz.T).T)+p0
            rec=np.empty(len(world),dtype=data_type);rec['xyz']=world;rec['intensity']=intensity;out_f.write(rec.tobytes());written+=len(rec)
            if frames%100==0:print('frames',frames,'points',written,'excluded',skipped,'outside',skipped_outside,flush=True)
        out_f.seek(count_pos);out_f.write(f'{written:020d}'.encode('ascii'))
    report={'method':'direct INSPVAX 1Hz interpolated pose; no second INS filter, no LiDAR pose graph, no high frequency IMU deskew','experimental':True,'frames_read':frames,'points_written':written,'points_excluded_bad_intervals':skipped,'points_outside_ins_coverage':skipped_outside,'ins_position_jumps_m':jump[invalid].tolist(),'ins_invalid_intervals':[[float(t[i]),float(t[i+1]),float(jump[i]),int(status[i]),int(status[i+1])] for i in np.flatnonzero(invalid)],'gps_utc_seconds_assumed':a.gps_utc_leap,'rotation_lidar_to_ins_candidate':ROT.tolist(),'translation_lidar_to_ins_m':[0,0,0],'warnings':['candidate rotation fitted to same bag, not calibrated','LiDAR-to-INS translation and antenna lever arm unverified','GPS/UTC time semantics not independently verified','position interpolation and 1Hz attitude, no high-rate gyro deskew','status 3 assumed usable per board report','output uncolored; no geometric or survey accuracy validation']};(a.output/'run_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()

