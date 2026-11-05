"""Picture of the camera holder + Pi 5 plate (assembled and individually)."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import generate_cam_pi_mount as M
exec(open(__file__.replace("render_cam_pi.py", "render_views.py")).read().split("parts={")[0])


def mesh(sh, tol=0.15):
    V, T = sh.tessellate(tol, 0.2); V = np.array([v.toTuple() for v in V]); P = V[np.array(T)]
    N = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); N /= np.linalg.norm(N, axis=1)[:, None] + 1e-12
    return P, N


def scene(parts, el, az, size):
    Ps, Ns, Cs = [], [], []
    for sh, c in parts:
        p, n = mesh(sh); Ps.append(p); Ns.append(n); Cs.append(np.tile(c, (len(p), 1)))
    P, N, C = np.vstack(Ps), np.vstack(Ns), np.vstack(Cs); R = rot_view(el, az); Q = P @ R.T
    lo, hi = Q.reshape(-1, 3).min(0), Q.reshape(-1, 3).max(0); sp = max(hi[0] - lo[0], hi[1] - lo[1]) * 1.08
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2; s = size / sp
    X = (Q[..., 0] - cx) * s + size / 2; Y = size / 2 - (Q[..., 1] - cy) * s; Z = Q[..., 2]
    zb = np.full((size, size), -np.inf); nb = np.zeros((size, size, 3)); cb = np.ones((size, size, 3)); Nv = N @ R.T
    for t in range(len(P)):
        x, y, z = X[t], Y[t], Z[t]
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(size - 1, np.ceil(x.max())))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(size - 1, np.ceil(y.max())))
        if x1 < x0 or y1 < y0: continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + .5, np.arange(y0, y1 + 1) + .5)
        d = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(d) < 1e-12: continue
        w0 = ((y[1] - y[2]) * (xs - x[2]) + (x[2] - x[1]) * (ys - y[2])) / d
        w1 = ((y[2] - y[0]) * (xs - x[2]) + (x[0] - x[2]) * (ys - y[2])) / d; w2 = 1 - w0 - w1
        mm = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6); zz = w0 * z[0] + w1 * z[1] + w2 * z[2]
        sub = zb[y0:y1 + 1, x0:x1 + 1]; u = mm & (zz > sub)
        sub[u] = zz[u]; nb[y0:y1 + 1, x0:x1 + 1][u] = Nv[t]; cb[y0:y1 + 1, x0:x1 + 1][u] = C[t]
    Lv = np.array([.35, .55, .75]); Lv /= np.linalg.norm(Lv); mask = np.isfinite(zb); img = np.ones((size, size, 3))
    img[mask] = cb[mask] * (.45 + .55 * np.abs((nb * Lv).sum(-1)))[mask][:, None]
    zf = np.where(mask, zb, -1e9); e = np.zeros((size, size), bool)
    for dy, dx in ((0, 1), (1, 0)):
        A = slice(0, size - dy), slice(0, size - dx); B = slice(dy, size), slice(dx, size)
        e[A] |= (mask[A] | mask[B]) & ((np.abs(zf[A] - zf[B]) > 12.0 / s) | ((np.abs((nb[A] * nb[B]).sum(-1)) < 0.9) & mask[A] & mask[B]))
    img[e] *= 0.3
    return img


if __name__ == "__main__":
    asm = M.build(); o = {c.name: (c.obj.val().moved(c.loc) if c.loc else c.obj.val()) for c in asm.children}
    col = {'beam_2020': (0.72, 0.72, 0.75), 'camera_holder_beam': (0.2, 0.45, 0.8), 'pi_camera_module3': (0.1, 0.55, 0.25),
           'pi5_top_plate': (0.95, 0.6, 0.15), 'raspberry_pi5_placeholder': (0.15, 0.5, 0.2), 'power_module_placeholder': (0.35, 0.35, 0.35)}
    parts = [(o[n], col.get(n, (0.95, 0.6, 0.15))) for n in o]
    fig = plt.figure(figsize=(15, 9.5))
    fig.suptitle('Camera holder (under the beam) + Raspberry Pi 5 plate (on top): each fixed with 2 M3 screws into T-nuts', fontsize=13, fontweight='bold')
    for pos, el, az, t in (([0, 0.47, 0.5, 0.47], 28, -55, 'Assembled on the beam, from above'), ([0.5, 0.47, 0.5, 0.47], -32, -55, 'From below: camera lens-down under the beam')):
        ax = fig.add_axes(pos); ax.imshow(scene(parts, el, az, 720)); ax.axis('off'); ax.set_title(t, loc='left')
    for pos, sh, el, az, c, t in (([0, 0.0, 0.33, 0.45], M.camera_holder_beam().val(), -35, -50, (0.2, 0.45, 0.8), 'Camera holder (from below): slots, cup, lips'),
                                  ([0.33, 0.0, 0.34, 0.45], M.pi_plate().val(), 45, -60, (0.95, 0.6, 0.15), 'Pi plate: Pi 5 left, power module right'),
                                  ([0.67, 0.0, 0.33, 0.45], M.pi_plate().val(), -40, -60, (0.95, 0.6, 0.15), 'Pi plate underside: lips + screw slots')):
        ax = fig.add_axes(pos); ax.imshow(scene([(sh, c)], el, az, 520)); ax.axis('off'); ax.set_title(t, loc='left', fontsize=10)
    plt.savefig(M.OUT / 'camera_pi_mount.png', dpi=85); print('ok')
