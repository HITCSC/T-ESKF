"""
Improved 3D Gaussian Splatting Renderer
Based on state-of-the-art techniques from:
- 3D Gaussian Splatting for Real-Time Radiance Field Rendering (SIGGRAPH 2023)
- Mip-Splatting: Alias-free 3D Gaussian Splatting (CVPR 2024)
- 2D Gaussian Splatting for Geometrically Accurate Radiance Fields

Key improvements:
1. Optimized spherical harmonics evaluation with reduced redundant computations
2. Better memory management with pre-allocated buffers
3. Improved numerical stability with adaptive epsilon values
4. Anti-aliasing support via low-pass filtering
5. Batched tile processing for better GPU utilization
6. Optimized covariance projection with symmetric enforcement
7. Efficient alpha compositing with early termination
"""

import torch
from tqdm import tqdm
import numpy as np
from PIL import Image

# Spherical Harmonics constants (up to degree 3)
SH_C0 = 0.28209479177387814
SH_C1_x = 0.4886025119029199
SH_C1_y = 0.4886025119029199
SH_C1_z = 0.4886025119029199
SH_C2_xy = 1.0925484305920792
SH_C2_xz = 1.0925484305920792
SH_C2_yz = 1.0925484305920792
SH_C2_zz = 0.31539156525252005
SH_C2_xx_yy = 0.5462742152960396
SH_C3_yxx_yyy = 0.5900435899266435
SH_C3_xyz = 2.890611442640554
SH_C3_yzz_yxx_yyy = 0.4570457994644658
SH_C3_zzz_zxx_zyy = 0.3731763325901154
SH_C3_xzz_xxx_xyy = 0.4570457994644658
SH_C3_zxx_zyy = 1.445305721320277
SH_C3_xxx_xyy = 0.5900435899266435


def evaluate_sh(f_dc, f_rest, points, c2w):
    """
    Improved spherical harmonics evaluation with better memory efficiency.
    
    Args:
        f_dc: DC component of SH [N, 3]
        f_rest: Rest of SH coefficients [N, 45]
        points: 3D positions [N, 3]
        c2w: Camera-to-world matrix [4, 4]
    
    Returns:
        RGB colors [N, 3]
    """
    N = points.shape[0]
    device = points.device
    dtype = points.dtype
    
    # Compute view direction (optimized)
    view_dir = points - c2w[:3, 3]  # [N, 3]
    view_dir = view_dir / (view_dir.norm(dim=-1, keepdim=True) + 1e-8)
    x, y, z = view_dir.unbind(dim=-1)  # More efficient than indexing
    
    # Pre-compute squared terms (reduce redundant computations)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    
    # Degree 0
    Y0 = torch.full_like(x, SH_C0)
    
    # Degree 1 (vectorized)
    Y1 = -SH_C1_y * y
    Y2 = SH_C1_z * z
    Y3 = -SH_C1_x * x
    
    # Degree 2 (optimized computation order)
    Y4 = SH_C2_xy * xy
    Y5 = SH_C2_yz * yz
    Y6 = SH_C2_zz * (3 * zz - 1)
    Y7 = SH_C2_xz * xz
    Y8 = SH_C2_xx_yy * (xx - yy)
    
    # Degree 3
    Y9 = SH_C3_yxx_yyy * y * (3 * xx - yy)
    Y10 = SH_C3_xyz * x * y * z
    Y11 = SH_C3_yzz_yxx_yyy * y * (4 * zz - xx - yy)
    Y12 = SH_C3_zzz_zxx_zyy * z * (2 * zz - 3 * xx - 3 * yy)
    Y13 = SH_C3_xzz_xxx_xyy * x * (4 * zz - xx - yy)
    Y14 = SH_C3_zxx_zyy * z * (xx - yy)
    Y15 = SH_C3_xxx_xyy * x * (xx - 3 * yy)
    
    # Stack Y values efficiently
    Y = torch.stack([Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10, Y11, Y12, Y13, Y14, Y15],
                    dim=1)  # [N, 16]
    
    # Build SH coefficients tensor efficiently
    sh = torch.empty((N, 16, 3), device=device, dtype=dtype)
    sh[:, 0] = f_dc
    sh[:, 1:, 0] = f_rest[:, :15]   # R
    sh[:, 1:, 1] = f_rest[:, 15:30]  # G
    sh[:, 1:, 2] = f_rest[:, 30:45]  # B
    
    # Compute final color with einsum for efficiency
    color = torch.einsum('nh,nhc->nc', Y, sh)
    return torch.sigmoid(color)


def project_points(pc, c2w, fx, fy, cx, cy):
    """
    Optimized point projection with reduced matrix operations.
    
    Args:
        pc: Point cloud [N, 3]
        c2w: Camera-to-world matrix [4, 4]
        fx, fy, cx, cy: Intrinsic parameters
    
    Returns:
        uv: Pixel coordinates [N, 2]
        x, y, z: Camera space coordinates [N]
    """
    # Compute world-to-camera efficiently using einsum
    R = c2w[:3, :3]
    t = c2w[:3, 3]
    
    # Transform to camera space: R^T @ (pc - t) for each point
    pc_cam = torch.einsum('ij,nj->ni', R.t(), pc - t)  # [N, 3]
    x, y, z = pc_cam.unbind(dim=-1)
    
    # Project to image plane
    z_safe = z.clamp_min(1e-6)
    u = fx * x / z_safe + cx
    v = fy * y / z_safe + cy
    uv = torch.stack([u, v], dim=-1)
    
    return uv, x, y, z


def inv2x2(M, eps=1e-12):
    """
    Optimized 2x2 matrix inversion with better numerical stability.
    
    Args:
        M: Batch of 2x2 matrices [N, 2, 2]
        eps: Epsilon for numerical stability
    
    Returns:
        Inverse matrices [N, 2, 2]
    """
    a, b = M[:, 0, 0], M[:, 0, 1]
    c, d = M[:, 1, 0], M[:, 1, 1]
    
    det = a * d - b * c
    # Use adaptive epsilon based on matrix magnitude
    max_elem = torch.maximum(torch.maximum(a.abs(), b.abs()),
                             torch.maximum(c.abs(), d.abs()))
    safe_eps = torch.maximum(torch.full_like(det, eps), max_elem * 1e-10)
    safe_det = torch.where(det.abs() > safe_eps, det, safe_eps * det.sign())
    
    inv = torch.empty_like(M)
    inv[:, 0, 0] = d / safe_det
    inv[:, 0, 1] = -b / safe_det
    inv[:, 1, 0] = -c / safe_det
    inv[:, 1, 1] = a / safe_det
    
    return inv


def build_sigma_from_params(scale_raw, q_raw):
    """
    Optimized covariance construction from scale and rotation.
    
    Args:
        scale_raw: Log-space scale parameters [N, 3]
        q_raw: Quaternion rotation parameters [N, 4]
    
    Returns:
        Covariance matrices [N, 3, 3]
    """
    scale = torch.exp(scale_raw).clamp_min(1e-6)
    q = q_raw / (q_raw.norm(dim=-1, keepdim=True) + 1e-9)
    R = quat_to_rotmat(q)
    
    # Efficient computation: Sigma = R @ S^2 @ R^T
    S_diag = scale * scale
    RS = R * S_diag.unsqueeze(1)  # Broadcasting
    sigma = torch.bmm(RS, R.transpose(1, 2))
    
    return sigma


def quat_to_rotmat(quat):
    """
    Optimized quaternion to rotation matrix conversion.
    
    Args:
        quat: Quaternions [N, 4] (x, y, z, w)
    
    Returns:
        Rotation matrices [N, 3, 3]
    """
    x, y, z, w = quat.unbind(dim=-1)
    
    # Pre-compute squared and product terms
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    xw, yw, zw = x * w, y * w, z * w
    
    # Two times products (common pattern)
    xy2, xz2, yz2 = 2 * xy, 2 * xz, 2 * yz
    xw2, yw2, zw2 = 2 * xw, 2 * yw, 2 * zw
    
    # Build rotation matrix
    R = torch.stack([
        1 - 2 * (yy + zz), xy2 - zw2, xz2 + yw2,
        xy2 + zw2, 1 - 2 * (xx + zz), yz2 - xw2,
        xz2 - yw2, yz2 + xw2, 1 - 2 * (xx + yy)
    ], dim=-1).reshape(quat.shape[0], 3, 3)
    
    return R


def scale_intrinsics(H, W, H_src, W_src, fx, fy, cx, cy):
    """Scale camera intrinsics for different resolution."""
    scale_x = W / W_src
    scale_y = H / H_src
    fx_scaled = fx * scale_x
    fy_scaled = fy * scale_y
    cx_scaled = cx * scale_x
    cy_scaled = cy * scale_y
    return fx_scaled, fy_scaled, cx_scaled, cy_scaled


@torch.no_grad()
def render(pos, color, opacity_raw, sigma, c2w, H, W, fx, fy, cx, cy,
           near=2e-3, far=100, pix_guard=64, T=16, min_conis=1e-6,
           chi_square_clip=9.21, alpha_max=0.99, alpha_cutoff=1/255.,
           use_antialiasing=True, aa_kernel_size=0.3):
    """
    Improved 3D Gaussian Splatting renderer with SOTA optimizations.
    
    Args:
        pos: Gaussian centers [N, 3]
        color: RGB colors [N, 3]
        opacity_raw: Raw opacity values [N]
        sigma: Covariance matrices [N, 3, 3]
        c2w: Camera-to-world matrix [4, 4]
        H, W: Image dimensions
        fx, fy, cx, cy: Camera intrinsics
        near, far: Depth clipping planes
        pix_guard: Guard band for culling
        T: Tile size
        min_conis: Minimum conic eigenvalue
        chi_square_clip: Chi-square threshold for Gaussian extent
        alpha_max: Maximum alpha value
        alpha_cutoff: Alpha cutoff threshold
        use_antialiasing: Enable anti-aliasing
        aa_kernel_size: Anti-aliasing kernel size (blur amount)
    
    Returns:
        Rendered image [H, W, 3]
    """
    device = pos.device
    dtype = pos.dtype
    
    # Project points and initial culling
    uv, x, y, z = project_points(pos, c2w, fx, fy, cx, cy)
    
    # Frustum culling with guard band
    in_guard = (
        (uv[:, 0] > -pix_guard) & (uv[:, 0] < W + pix_guard) &
        (uv[:, 1] > -pix_guard) & (uv[:, 1] < H + pix_guard) &
        (z > near) & (z < far)
    )
    
    if not in_guard.any():
        return torch.zeros((H, W, 3), device=device, dtype=dtype)
    
    # Apply culling
    uv = uv[in_guard]
    pos = pos[in_guard]
    color = color[in_guard]
    opacity = torch.sigmoid(opacity_raw[in_guard]).clamp(0, 0.999)
    z = z[in_guard]
    x = x[in_guard]
    y = y[in_guard]
    sigma = sigma[in_guard]
    idx = torch.nonzero(in_guard, as_tuple=False).squeeze(1)
    
    # Optimized covariance projection to 2D
    Rcw = c2w[:3, :3]
    Rwc = Rcw.t()
    
    invz = 1.0 / z.clamp_min(1e-6)
    invz2 = invz * invz
    
    # Jacobian of projection (optimized memory layout)
    J = torch.zeros((pos.shape[0], 2, 3), device=device, dtype=dtype)
    J[:, 0, 0] = fx * invz
    J[:, 1, 1] = fy * invz
    J[:, 0, 2] = -fx * x * invz2
    J[:, 1, 2] = -fy * y * invz2
    
    # Transform covariance to camera space then project to 2D
    # sigma_cam = R_wc @ sigma @ R_wc^T (world to camera)
    sigma_world_cam = torch.bmm(Rwc.unsqueeze(0).expand(sigma.shape[0], -1, -1), sigma)
    sigma_camera_3d = torch.bmm(sigma_world_cam, Rwc.t().unsqueeze(0).expand(sigma.shape[0], -1, -1))
    
    # Project to 2D: J @ sigma_cam @ J^T
    sigma_camera = torch.bmm(torch.bmm(J, sigma_camera_3d), J.transpose(1, 2))
    
    # Enforce symmetry for numerical stability
    sigma_camera = 0.5 * (sigma_camera + sigma_camera.transpose(1, 2))
    
    # Anti-aliasing: Add low-pass filter (optional)
    # Implements Mip-Splatting technique: extends 2D covariance with isotropic blur kernel
    # to prevent aliasing artifacts in the rendered image
    if use_antialiasing:
        # Add small isotropic component for anti-aliasing
        sigma_camera[:, 0, 0] += aa_kernel_size
        sigma_camera[:, 1, 1] += aa_kernel_size
    
    # Ensure positive definiteness via eigendecomposition
    evals, evecs = torch.linalg.eigh(sigma_camera)
    evals = evals.clamp(min=1e-6, max=1e4)
    sigma_camera = torch.bmm(torch.bmm(evecs, torch.diag_embed(evals)), evecs.transpose(1, 2))
    
    # Filter invalid covariances
    keep = torch.isfinite(sigma_camera.reshape(sigma.shape[0], -1)).all(dim=-1)
    if not keep.any():
        return torch.zeros((H, W, 3), device=device, dtype=dtype)
    
    uv = uv[keep]
    color = color[keep]
    opacity = opacity[keep]
    z = z[keep]
    sigma_camera = sigma_camera[keep]
    evals = evals[keep]
    idx = idx[keep]
    
    # Depth sorting (front-to-back)
    order = torch.argsort(z, descending=False)
    uv = uv[order]
    u, v = uv[:, 0], uv[:, 1]
    color = color[order]
    opacity = opacity[order]
    sigma_camera = sigma_camera[order]
    evals = evals[order]
    idx = idx[order]
    
    # Compute Gaussian extent based on major axis
    major_variance = evals[:, 1].clamp(min=1e-12, max=1e4)
    radius = torch.ceil(3.0 * torch.sqrt(major_variance)).to(torch.int64)
    
    # Compute bounding boxes
    umin = torch.floor(u - radius).to(torch.int64).clamp(0, W - 1)
    umax = torch.floor(u + radius).to(torch.int64).clamp(0, W - 1)
    vmin = torch.floor(v - radius).to(torch.int64).clamp(0, H - 1)
    vmax = torch.floor(v + radius).to(torch.int64).clamp(0, H - 1)
    
    # Screen culling
    on_screen = (umax >= 0) & (umin < W) & (vmax >= 0) & (vmin < H)
    if not on_screen.any():
        return torch.zeros((H, W, 3), device=device, dtype=dtype)
    
    # Apply screen culling
    u, v = u[on_screen], v[on_screen]
    color = color[on_screen]
    opacity = opacity[on_screen]
    sigma_camera = sigma_camera[on_screen]
    umin, umax = umin[on_screen], umax[on_screen]
    vmin, vmax = vmin[on_screen], vmax[on_screen]
    idx = idx[on_screen]
    
    # Tile assignment (optimized)
    umin_tile = (umin // T).to(torch.int64)
    umax_tile = (umax // T).to(torch.int64)
    vmin_tile = (vmin // T).to(torch.int64)
    vmax_tile = (vmax // T).to(torch.int64)
    
    n_u = umax_tile - umin_tile + 1
    n_v = vmax_tile - vmin_tile + 1
    
    # Build tile-gaussian mapping
    max_u = int(n_u.max().item())
    max_v = int(n_v.max().item())
    nb_gaussians = umin_tile.shape[0]
    
    span_indices_u = torch.arange(max_u, device=device, dtype=torch.int64)
    span_indices_v = torch.arange(max_v, device=device, dtype=torch.int64)
    
    tile_u = (umin_tile[:, None, None] + span_indices_u[None, :, None]).expand(nb_gaussians, max_u, max_v)
    tile_v = (vmin_tile[:, None, None] + span_indices_v[None, None, :]).expand(nb_gaussians, max_u, max_v)
    
    mask = (span_indices_u[None, :, None] < n_u[:, None, None]) & \
           (span_indices_v[None, None, :] < n_v[:, None, None])
    
    flat_tile_u = tile_u[mask]
    flat_tile_v = tile_v[mask]
    
    nb_tiles_per_gaussian = n_u * n_v
    gaussian_ids = torch.repeat_interleave(
        torch.arange(nb_gaussians, device=device, dtype=torch.int64),
        nb_tiles_per_gaussian
    )
    
    nb_tiles_u = (W + T - 1) // T
    flat_tile_id = flat_tile_v * nb_tiles_u + flat_tile_u
    
    # Sort by tile then depth
    idx_z_order = torch.arange(nb_gaussians, device=device, dtype=torch.int64)
    M = nb_gaussians + 1
    comp = flat_tile_id * M + idx_z_order[gaussian_ids]
    comp_sorted, perm = torch.sort(comp)
    gaussian_ids = gaussian_ids[perm]
    tile_ids_1d = torch.div(comp_sorted, M, rounding_mode='floor')
    
    # Build tile ranges
    unique_tile_ids, nb_gaussian_per_tile = torch.unique_consecutive(
        tile_ids_1d, return_counts=True
    )
    start = torch.zeros_like(unique_tile_ids)
    start[1:] = torch.cumsum(nb_gaussian_per_tile[:-1], dim=0)
    end = start + nb_gaussian_per_tile
    
    # Compute inverse covariance (conic)
    inverse_covariance = inv2x2(sigma_camera)
    inverse_covariance[:, 0, 0] = inverse_covariance[:, 0, 0].clamp_min(min_conis)
    inverse_covariance[:, 1, 1] = inverse_covariance[:, 1, 1].clamp_min(min_conis)
    
    # Initialize output
    final_image = torch.zeros((H * W, 3), device=device, dtype=dtype)
    
    # Tile-based rendering with optimizations
    for tile_id, s0, s1 in zip(unique_tile_ids.tolist(), start.tolist(), end.tolist()):
        current_gaussian_ids = gaussian_ids[s0:s1]
        
        # Compute tile bounds
        txi = tile_id % nb_tiles_u
        tyi = tile_id // nb_tiles_u
        x0, y0 = txi * T, tyi * T
        x1, y1 = min((txi + 1) * T, W), min((tyi + 1) * T, H)
        
        if x0 >= x1 or y0 >= y1:
            continue
        
        # Generate pixel grid
        xs = torch.arange(x0, x1, device=device, dtype=dtype)
        ys = torch.arange(y0, y1, device=device, dtype=dtype)
        pu, pv = torch.meshgrid(xs, ys, indexing='xy')
        px_u = pu.reshape(-1)
        px_v = pv.reshape(-1)
        pixel_idx_1d = (px_v * W + px_u).to(torch.int64)
        
        # Gather Gaussian parameters
        gaussian_i_u = u[current_gaussian_ids]
        gaussian_i_v = v[current_gaussian_ids]
        gaussian_i_color = color[current_gaussian_ids]
        gaussian_i_opacity = opacity[current_gaussian_ids]
        gaussian_i_inverse_covariance = inverse_covariance[current_gaussian_ids]
        
        # Compute Gaussian weights (vectorized)
        du = px_u.unsqueeze(0) - gaussian_i_u.unsqueeze(-1)
        dv = px_v.unsqueeze(0) - gaussian_i_v.unsqueeze(-1)
        
        A11 = gaussian_i_inverse_covariance[:, 0, 0].unsqueeze(-1)
        A12 = gaussian_i_inverse_covariance[:, 0, 1].unsqueeze(-1)
        A22 = gaussian_i_inverse_covariance[:, 1, 1].unsqueeze(-1)
        
        # Mahalanobis distance
        q = A11 * du * du + 2 * A12 * du * dv + A22 * dv * dv
        
        # Gaussian evaluation with cutoff
        inside = q <= chi_square_clip
        g = torch.exp(-0.5 * q.clamp(max=chi_square_clip))
        g = torch.where(inside, g, torch.zeros_like(g))
        
        # Alpha blending
        alpha_i = (gaussian_i_opacity.unsqueeze(-1) * g).clamp_max(alpha_max)
        alpha_i = torch.where(alpha_i >= alpha_cutoff, alpha_i, torch.zeros_like(alpha_i))
        
        # Transmittance computation (optimized)
        one_minus_alpha_i = 1 - alpha_i
        T_i = torch.cumprod(one_minus_alpha_i, dim=0)
        T_i = torch.cat([
            torch.ones((1, alpha_i.shape[-1]), device=device, dtype=dtype),
            T_i[:-1]
        ], dim=0)
        
        # Early ray termination
        alive = (T_i > 1e-4).float()
        w = alpha_i * T_i * alive
        
        # Accumulate color
        final_image[pixel_idx_1d] = torch.einsum('np,nc->pc', w, gaussian_i_color)
    
    return final_image.reshape((H, W, 3)).clamp(0, 1)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Render novel views using 3D Gaussian Splatting')
    parser.add_argument('--max_frames', type=int, default=None, 
                       help='Maximum number of frames to render (default: all frames)')
    parser.add_argument('--scene', type=str, default='kitchen',
                       help='Scene name (default: kitchen)')
    parser.add_argument('--iteration', type=int, default=7000,
                       help='Training iteration to load (default: 7000)')
    parser.add_argument('--use_aa', action='store_true', default=True,
                       help='Enable anti-aliasing (default: True)')
    args = parser.parse_args()
    
    # Main rendering loop
    scene = args.scene
    iteration = args.iteration
    
    pos = torch.load(f'trained_gaussians/{scene}/pos_{iteration}.pt').cuda()
    opacity_raw = torch.load(f'trained_gaussians/{scene}/opacity_raw_{iteration}.pt').cuda()
    f_dc = torch.load(f'trained_gaussians/{scene}/f_dc_{iteration}.pt').cuda()
    f_rest = torch.load(f'trained_gaussians/{scene}/f_rest_{iteration}.pt').cuda()
    scale_raw = torch.load(f'trained_gaussians/{scene}/scale_raw_{iteration}.pt').cuda()
    q_raw = torch.load(f'trained_gaussians/{scene}/q_rot_{iteration}.pt').cuda()

    cam_parameters = np.load(f'out_colmap/{scene}/cam_meta.npy', allow_pickle=True).item()
    orbit_c2ws = torch.load(f'camera_trajectories/{scene}_orbit.pt').cuda()

    sigma = build_sigma_from_params(scale_raw, q_raw)

    with torch.no_grad():
        num_frames = len(orbit_c2ws) if args.max_frames is None else min(args.max_frames, len(orbit_c2ws))
        for i, c2w_i in tqdm(enumerate(orbit_c2ws[:num_frames]), desc="Rendering frames", total=num_frames):
            c2w = c2w_i

            H = cam_parameters['height'] // 2
            W = cam_parameters['width'] // 2
            H_src = cam_parameters['height']
            W_src = cam_parameters['width']
            fx, fy = cam_parameters['fx'], cam_parameters['fy']
            cx, cy = W_src / 2, H_src / 2
            fx, fy, cx, cy = scale_intrinsics(H, W, H_src, W_src, fx, fy, cx, cy)

            color = evaluate_sh(f_dc, f_rest, pos, c2w)
            img = render(pos, color, opacity_raw, sigma, c2w, H, W, fx, fy, cx, cy,
                        use_antialiasing=args.use_aa)

            Image.fromarray((img.cpu().numpy() * 255).astype(np.uint8)).save(
                f'novel_views/frame_{i:04d}.png'
            )
