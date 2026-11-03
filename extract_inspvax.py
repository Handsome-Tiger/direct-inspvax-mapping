"""Extract CRC-verified INSPVAX poses from the board's raw serial dump."""
import argparse,csv,struct
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def crc32(data):
 c=0
 for b in data:
  c^=b
  for _ in range(8):c=(c>>1)^0xedb88320 if c&1 else c>>1
 return c&0xffffffff
def ecef(lat,lon,h):
 lat,lon=np.radians([lat,lon]);a=6378137.;e2=6.6943799901413165e-3;n=a/np.sqrt(1-e2*np.sin(lat)**2)
 return np.array([(n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-e2)+h)*np.sin(lat)])
def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=a.raw.read_bytes();i=0;rows=[]
 while i+10<=len(d):
  j=d.find(b'\xaa\x44',i)
  if j<0 or j+10>len(d):break
  kind=d[j+2]
  if kind==0x13:n=56;h=52
  elif kind==0x12:
   h=d[j+3];n=h+struct.unpack_from('<H',d,j+8)[0]+4
   if not 20<=h<=64 or not h+4<n<=4164:i=j+1;continue
  else:i=j+1;continue
  if j+n>len(d):break
  f=d[j:j+n]
  if crc32(f[:-4])!=struct.unpack_from('<I',f,n-4)[0]:i=j+1;continue
  if kind==0x12 and struct.unpack_from('<H',f,4)[0]==1465 and n>=h+130:
   week=struct.unpack_from('<H',f,14)[0];sow=struct.unpack_from('<I',f,16)[0]/1000.;st,pt=struct.unpack_from('<II',f,h);lat,lon,msl=struct.unpack_from('<ddd',f,h+8);und=struct.unpack_from('<f',f,h+32)[0];vn,ve,vu=struct.unpack_from('<ddd',f,h+36);roll,pitch,az=struct.unpack_from('<ddd',f,h+60);std=struct.unpack_from('<9f',f,h+84);rows.append([week,sow,st,pt,lat,lon,msl,und,msl+und,ve,vn,vu,roll,pitch,az,*std])
  i=j+n
 if len(rows)<2:raise ValueError('Insufficient verified INSPVAX frames')
 arr=np.asarray(rows);lat0,lon0=np.radians(arr[0,4:6]);basis=np.array([[-np.sin(lon0),np.cos(lon0),0],[-np.sin(lat0)*np.cos(lon0),-np.sin(lat0)*np.sin(lon0),np.cos(lat0)],[np.cos(lat0)*np.cos(lon0),np.cos(lat0)*np.sin(lon0),np.sin(lat0)]]);origin=ecef(*arr[0,[4,5,8]]);xyz=np.asarray([basis@(ecef(*r[[4,5,8]])-origin) for r in arr]);q=Rotation.from_euler('xyz',np.column_stack([arr[:,12],-arr[:,13],90-arr[:,14]]),degrees=True).as_quat()
 names=['gps_week','gps_seconds','ins_status','pos_type','lat_deg','lon_deg','height_msl_m','undulation_m','ellipsoid_height_m','velocity_east_mps','velocity_north_mps','velocity_up_mps','roll_deg','pitch_deg','azimuth_deg','lat_std_m','lon_std_m','height_std_m','north_velocity_std_mps','east_velocity_std_mps','up_velocity_std_mps','roll_std_deg','pitch_std_deg','azimuth_std_deg','east_m','north_m','up_m','qx','qy','qz','qw']
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x',encoding='utf-8',newline='') as f:w=csv.writer(f);w.writerow(names);w.writerows(np.column_stack([arr,xyz,q]))
 print('CRC-valid INSPVAX frames',len(rows),'output',a.output)
if __name__=='__main__':main()

