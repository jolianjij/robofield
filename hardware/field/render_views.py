"""Render 6-view sheets (top, front, side, 2x 3D, bottom) of every printed field part.

    python hardware/field/render_views.py      # -> hardware/field/out/views_<part>.png
"""
import numpy as np
def rot_view(el, az):
    e, a = np.radians(el), np.radians(az)
    d = np.array([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)])   # toward viewer
    up = np.array([0, 0, 1.0]) if abs(el) < 89 else np.array([-np.sin(a), np.cos(a), 0]) * (1 if el > 0 else -1)
    if abs(el) >= 89: up = np.array([0, 1.0, 0]) if el > 0 else np.array([0, -1.0, 0])
    r = np.cross(up, d); r /= np.linalg.norm(r); u = np.cross(d, r)
    return np.stack([r, u, d])
def render(P, N, el, az, size=420, color=(0.95, 0.55, 0.15), bounds=None, span=None):
    R = rot_view(el, az)
    Q = P @ R.T                                   # (n,3,3) view coords: x right, y up, z toward viewer
    if bounds is None:
        allp = Q.reshape(-1, 3); lo, hi = allp.min(0), allp.max(0)
    else: lo, hi = bounds
    span = span or max(hi[0]-lo[0], hi[1]-lo[1]) * 1.1; cx, cy = (lo[0]+hi[0])/2, (lo[1]+hi[1])/2
    s = size / span
    X = (Q[..., 0]-cx)*s + size/2; Y = size/2 - (Q[..., 1]-cy)*s; Z = Q[..., 2]
    zb = np.full((size, size), -np.inf); nb = np.zeros((size, size, 3)); idb = np.full((size, size), -1)
    Nv = N @ R.T
    for t in range(len(P)):
        x, y, z = X[t], Y[t], Z[t]
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(size-1, np.ceil(x.max())))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(size-1, np.ceil(y.max())))
        if x1 < x0 or y1 < y0: continue
        xs, ys = np.meshgrid(np.arange(x0, x1+1)+0.5, np.arange(y0, y1+1)+0.5)
        d = (y[1]-y[2])*(x[0]-x[2]) + (x[2]-x[1])*(y[0]-y[2])
        if abs(d) < 1e-12: continue
        w0 = ((y[1]-y[2])*(xs-x[2]) + (x[2]-x[1])*(ys-y[2]))/d
        w1 = ((y[2]-y[0])*(xs-x[2]) + (x[0]-x[2])*(ys-y[2]))/d
        w2 = 1-w0-w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any(): continue
        zz = w0*z[0]+w1*z[1]+w2*z[2]
        sub = zb[y0:y1+1, x0:x1+1]; upd = m & (zz > sub)
        sub[upd] = zz[upd]; nb[y0:y1+1, x0:x1+1][upd] = Nv[t]; idb[y0:y1+1, x0:x1+1][upd] = t
    img = np.ones((size, size, 3))
    mask = np.isfinite(zb)
    L = np.array([0.35, 0.55, 0.75]); L /= np.linalg.norm(L)
    sh = 0.45 + 0.55*np.clip(np.abs((nb*L).sum(-1)), 0, 1)
    img[mask] = np.array(color)[None]*sh[mask][:, None]
    # feature edges: depth jumps and normal changes
    zf = np.where(mask, zb, -1e9)
    edge = np.zeros((size, size), bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = slice(0, size-dy), slice(0, size-dx); b = slice(dy, size), slice(dx, size)
        dz = np.abs(zf[a]-zf[b]) > 1.5
        dn = (nb[a]*nb[b]).sum(-1) < 0.9
        mm = mask[a] | mask[b]
        e = mm & (dz | (dn & mask[a] & mask[b]))
        edge[a] |= e
    img[edge] = img[edge]*0.25
    return img
import sys; sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import generate_field_parts as g
parts={'wall_module':(g.wall_module,'Wall module (x16)'),'splice_plate':(g.splice_plate,'Splice plate (x11)'),
       'tower_base':(g.tower_base,'Tower base (x4)'),'tower_cap':(g.tower_cap,'Tower cap (x4)'),
       'mast_clamp':(g.mast_clamp,'Mast clamp (x1)'),'mast_elbow':(g.mast_elbow,'Mast elbow (x1)'),'brace_clamp':(g.brace_clamp,'Brace clamp (x2)'),'camera_holder':(g.camera_holder,'Camera holder (x1)')}
views=[('Top',90,-90),('Front',0,-90),('Right side',0,0),('3D view',28,-55),('3D view from behind',28,125),('Bottom',-90,-90)]
for key,(fn,title) in parts.items():
    sh=fn().val(); bb=sh.BoundingBox()
    V,T=sh.tessellate(0.1,0.1); V=np.array([v.toTuple() for v in V]); P=V[np.array(T)]
    N=np.cross(P[:,1]-P[:,0],P[:,2]-P[:,0]); N/=np.linalg.norm(N,axis=1)[:,None]+1e-12
    fig,axs=plt.subplots(2,3,figsize=(13,8.6))
    fig.suptitle(f'{title}  —  {bb.xlen:.0f} × {bb.ylen:.0f} × {bb.zlen:.0f} mm',fontsize=15,fontweight='bold')
    for ax,(name,el,az) in zip(axs.flat,views):
        ax.imshow(render(P,N,el,az,size=460,span=1.05*(bb.xlen**2+bb.ylen**2+bb.zlen**2)**0.5)); ax.set_title(name,fontsize=12); ax.axis('off')
    plt.tight_layout(rect=(0,0,1,0.95)); g.OUT.mkdir(parents=True, exist_ok=True); plt.savefig(g.OUT / f'views_{key}.png', dpi=90); plt.close()
    print('ok',key,len(P))
