"""Demo: đưa lidar, odom, landmark về CÙNG 1 hệ tọa độ (odom), so sánh landmark cũ vs đã sửa frame + range."""
import sys, math, numpy as np, cv2, rosbag2_py
from collections import defaultdict
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from ultralytics import YOLO
BAG, MODEL, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
def load(topic, cls, f, step=1):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic])); raw=[]; k=0
    while r.has_next():
        _,d,_=r.read_next(); raw.append((d,k)); k+=1
    out=[]; tot=defaultdict(int); cnt=defaultdict(int); msgs=[]
    for d,k in raw:
        m=deserialize_message(d,cls); msgs.append((m.header.stamp.sec,m,k))
    for s,_,_ in msgs: tot[s]+=1
    for s,m,k in msgs:
        t=s+cnt[s]/tot[s]; cnt[s]+=1
        if k%step==0: out.append((t,f(m)))
    return out
odom=load('/odom',Odometry,lambda m:(m.pose.pose.position.x,m.pose.pose.position.y,2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)))[:-1]  # bỏ tin cuối hỏng
OT=np.array([o[0] for o in odom]); OP=np.array([o[1] for o in odom])
def pose(t): i=min(np.searchsorted(OT,t),len(OT)-1); return OP[i]
scans=load('/scan',LaserScan,lambda m:np.array(m.ranges,float))
ST=np.array([s[0] for s in scans]); n=len(scans[0][1]); ANG=np.arange(n)*2*math.pi/n
# 1. Map lidar chiếu theo odom (laser 0° = trục x base_link)
P=[]
for t,r in scans[::3]:
    x,y,th=pose(t); m=np.isfinite(r)&(r>0.15)&(r<11.5)
    a=ANG[m]+th; P.append(np.c_[x+r[m]*np.cos(a), y+r[m]*np.sin(a)])
P=np.vstack(P); tree=cKDTree(P[::5]); print('lidar pts', len(P), flush=True)
# 2. Landmark: cũ vs sửa
FX,CX=615.0,320.0
H_OLD={0:0.85,1:0.60,2:2.00,3:0.85}; H_NEW={0:1.8,1:1.6,2:2.3,3:1.0}
model=YOLO(MODEL); imgs=load('/camera/image_raw/compressed',CompressedImage,lambda m:bytes(m.data),step=10)
old=[]; new=[]
for t,buf in imgs:
    img=cv2.imdecode(np.frombuffer(buf,np.uint8),1); res=model(img,verbose=False,conf=0.5,iou=0.4)[0]
    if res.boxes is None or len(res.boxes)==0: continue
    x,y,th=pose(t); si=min(np.searchsorted(ST,t),len(ST)-1); sc=scans[si][1]
    for b,c in zip(res.boxes.xyxy.cpu().numpy(),res.boxes.cls.cpu().numpy().astype(int)):
        if c not in H_OLD: continue
        x1,y1,x2,y2=b; h=y2-y1
        if h<10: continue
        bear=math.atan2(-((x1+x2)/2-CX),FX)
        # CŨ: như code hiện tại (camera = +x base_link, chiều cao cũ)
        d=FX*H_OLD[c]/h; a=th+bear; old.append((x+d*math.cos(a),y+d*math.sin(a),c))
        # MỚI: camera xoay pi; range lidar trong khoảng bbox nếu có, không thì mono với chiều cao mới
        b1=math.atan2(-(x1-CX),FX); b2=math.atan2(-(x2-CX),FX)
        idx=[int(round(((bb+math.pi)%(2*math.pi))/(2*math.pi)*n))%n for bb in np.linspace(b2,b1,9)]
        v=sc[idx]; v=v[np.isfinite(v)&(v>0.15)&(v<11.5)]
        edge = y1<3 or y2>477
        if v.size>=3: d=float(np.median(v)); src=1
        elif not edge: d=FX*H_NEW[c]/h; src=0
        else: continue
        a=th+math.pi+bear; new.append((x+d*math.cos(a),y+d*math.sin(a),c,src))
old=np.array(old); new=np.array(new)
do,_=tree.query(old[:,:2]); dn,_=tree.query(new[:,:2])
print('detections old %d new %d (lidar range %d%%)' % (len(old),len(new),100*new[:,3].mean()))
print('distance landmark -> nearest lidar wall point: OLD median %.2f m (<0.5m: %.0f%%) | NEW median %.2f m (<0.5m: %.0f%%)' % (np.median(do),100*np.mean(do<0.5),np.median(dn),100*np.mean(dn<0.5)))
m=new[:,3]==1; print('   NEW lidar-range only: median %.2f m ; NEW mono-only: median %.2f m' % (np.median(dn[m]),np.median(dn[~m])))
# 3. Vẽ
allp=np.vstack([P[::20],old[:,:2],new[:,:2]]); lo=np.percentile(allp,0.5,axis=0)-3; hi=np.percentile(allp,99.5,axis=0)+3
s=1400/max(hi-lo); W,Hh=int((hi-lo)[0]*s)+1,int((hi-lo)[1]*s)+1
def canvas(title):
    im=np.full((Hh,W,3),255,np.uint8)
    q=np.int32(np.c_[(P[:,0]-lo[0])*s,(hi[1]-P[:,1])*s]); q=q[(q[:,0]>=0)&(q[:,0]<W)&(q[:,1]>=0)&(q[:,1]<Hh)]
    im[q[:,1],q[:,0]]=(150,150,150)
    tr=np.int32(np.c_[(OP[:,0]-lo[0])*s,(hi[1]-OP[:,1])*s]); cv2.polylines(im,[tr.reshape(-1,1,2)],False,(200,60,0),2)
    cv2.putText(im,title,(15,35),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,0),2); return im
COL={0:(230,160,0),1:(40,40,220),2:(0,150,255),3:(200,0,200)}
for name,L,dd in (('old',old,do),('new',new,dn)):
    im=canvas(('OLD: camera = +x base_link, old heights' if name=='old' else 'NEW: camera rotated pi, lidar range')+'  | median dist to wall %.2f m' % np.median(dd))
    for p in L: cv2.circle(im,(int((p[0]-lo[0])*s),int((hi[1]-p[1])*s)),3,COL[int(p[2])],-1)
    cv2.imwrite(OUT+'_'+name+'.png',im)
np.savez(OUT+'.npz',P=P[::5],old=old,new=new,odom=OP)
