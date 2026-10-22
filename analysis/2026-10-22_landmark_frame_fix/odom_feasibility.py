"""Thử nghiệm: dựng lại odom từ lidar (ICP) và video (VO), đánh giá bằng độ khép vòng đầu-cuối."""
import sys, math, json, numpy as np, cv2, rosbag2_py
from collections import defaultdict
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
BAG, OUT = sys.argv[1], sys.argv[2]
def load(topic, cls, f, step=1):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic])); raw=[]; k=0
    while r.has_next():
        _,d,_=r.read_next(); m=deserialize_message(d,cls); raw.append((m.header.stamp.sec, f(m) if k%step==0 else None)); k+=1
    tot=defaultdict(int); cnt=defaultdict(int)
    for s,_ in raw: tot[s]+=1
    out=[]
    for s,v in raw:
        t=s+cnt[s]/tot[s]; cnt[s]+=1   # dựng lại thời gian: chia đều trong mỗi giây
        if v is not None: out.append((t,v))
    return out
odom=load('/odom',Odometry,lambda m:(m.pose.pose.position.x,m.pose.pose.position.y,2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)))
OT=np.array([o[0] for o in odom]); OP=np.array([o[1] for o in odom])
def odom_at(t):
    i=min(np.searchsorted(OT,t),len(OT)-1); return OP[i]
print('odom', len(odom), flush=True)
# ---------- Lidar ICP yaw ----------
scans=load('/scan',LaserScan,lambda m:np.array(m.ranges,float))
n=len(scans[0][1]); ang=np.arange(n)*2*math.pi/n
def pts(r):
    m=np.isfinite(r)&(r>0.15)&(r<11.5); return np.c_[r[m]*np.cos(ang[m]), r[m]*np.sin(ang[m])]
def icp(A,B,it=30):
    if len(A)<30 or len(B)<30: return None
    tree=cKDTree(A); R=np.eye(2); t=np.zeros(2)
    for _ in range(it):
        Bt=B@R.T+t; d,idx=tree.query(Bt); k=d<0.4
        if k.sum()<25: return None
        P=Bt[k]; Q=A[idx[k]]; mp,mq=P.mean(0),Q.mean(0)
        U,_,Vt=np.linalg.svd((P-mp).T@(Q-mq)); dR=(U@Vt).T
        if np.linalg.det(dR)<0: return None
        R=dR@R; t=dR@t+(mq-dR@mp)
    return math.atan2(R[1,0],R[0,0])
lid=[]  # (t0,t1,dyaw_icp)
for i in range(len(scans)-1):
    (t0,r0),(t1,r1)=scans[i],scans[i+1]
    y=icp(pts(r0),pts(r1)); lid.append((t0,t1,y))
print('icp done, fail', sum(1 for x in lid if x[2] is None), '/', len(lid), flush=True)
# ---------- Camera VO yaw ----------
K=np.array([[615.,0,320],[0,615.,240],[0,0,1]])
imgs=load('/camera/image_raw/compressed',CompressedImage,lambda m:cv2.imdecode(np.frombuffer(bytes(m.data),np.uint8),0),step=3)
orb=cv2.ORB_create(1500); bf=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
feat=[orb.detectAndCompute(g,None) for _,g in imgs]
vo=[]
for i in range(len(imgs)-1):
    (k0,d0),(k1,d1)=feat[i],feat[i+1]; y=None
    if d0 is not None and d1 is not None and len(k0)>50 and len(k1)>50:
        mt=bf.match(d0,d1)
        if len(mt)>40:
            p0=np.float32([k0[m.queryIdx].pt for m in mt]); p1=np.float32([k1[m.trainIdx].pt for m in mt])
            E,msk=cv2.findEssentialMat(p0,p1,K,cv2.RANSAC,0.999,1.0)
            if E is not None and E.shape==(3,3):
                ninl,R,tv,_=cv2.recoverPose(E,p0,p1,K,mask=msk)
                if ninl>30: y=math.atan2(R[0,2],R[2,2])
    vo.append((imgs[i][0],imgs[i+1][0],y))
print('vo done, fail', sum(1 for x in vo if x[2] is None), '/', len(vo), flush=True)
# ---------- So khớp dấu với odom ở các khúc quay rõ ----------
def odom_dyaw(t0,t1):
    a,b=odom_at(t0),odom_at(t1); return math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
def win_sum(lst,t0,t1): return sum(y for a,b,y in lst if y is not None and a>=t0 and b<=t1)
W=np.arange(OT[0],OT[-1]-5,5.0)
cmp=np.array([(odom_dyaw(w,w+5),win_sum(lid,w,w+5),win_sum(vo,w,w+5)) for w in W])
s_vo=np.sign(np.corrcoef(cmp[:,1],cmp[:,2])[0,1]) or 1.0
print('corr 5s windows: odom-icp %.2f odom-vo %.2f icp-vo %.2f (vo sign %+d)' % (np.corrcoef(cmp[:,0],cmp[:,1])[0,1], np.corrcoef(cmp[:,0],s_vo*cmp[:,2])[0,1], np.corrcoef(cmp[:,1],s_vo*cmp[:,2])[0,1], s_vo), flush=True)
# ---------- Dựng quỹ đạo: bước thời gian theo scan ----------
def build(yaw_src):
    x=y=0.0; th=OP[0,2]+math.pi  # hướng đi thật = yaw odom + pi
    traj=[(x,y)]
    for (t0,t1,yi) in lid:
        a,b=odom_at(t0),odom_at(t1); ds=math.hypot(b[0]-a[0],b[1]-a[1])
        dth={'odom':odom_dyaw(t0,t1),'lidar':yi,'video':None,'fused':None}[yaw_src] if yaw_src in('odom','lidar') else None
        if yaw_src in('video','fused'):
            v=win_sum(vo,t0-0.05,t1+0.05) if False else None
        if dth is None and yaw_src=='lidar': dth=odom_dyaw(t0,t1)
        if yaw_src in ('video','fused'):
            vv=[s_vo*yy for a_,b_,yy in vo if yy is not None and a_>=t0-1e-9 and a_<t1]
            dv=sum(vv) if vv else None
            if yaw_src=='video': dth=dv if dv is not None else odom_dyaw(t0,t1)
            else:
                cands=[c for c in (yi,dv) if c is not None]; dth=float(np.mean(cands)) if cands else odom_dyaw(t0,t1)
        th+=dth; x+=ds*math.cos(th); y+=ds*math.sin(th); traj.append((x,y))
    return np.array(traj)
res={}
for src in ('odom','lidar','video','fused'):
    T=build(src); L=np.sum(np.hypot(*np.diff(T,axis=0).T)); e=math.hypot(*(T[-1]-T[0]))
    res[src]=T; print('%-6s path %.1f m  END-START closure error %.2f m (%.2f%% of path)' % (src,L,e,100*e/L), flush=True)
raw=OP[:,:2]; print('raw /odom in bag: closure error %.2f m' % math.hypot(*(raw[-1]-raw[0])))
np.savez(OUT, **res, raw=raw)
