// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Renders the application icon at every size the installers need (run by
//! packaging/icons/build.sh): a rounded tile with an intersection and a signal head.
//!
//! usage: make_icons <out-dir>

use intersection_lab::gui::shot::write_png;

type Rgba = [f32; 4];

fn mix(a: Rgba, b: Rgba, t: f32) -> Rgba {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, a[3] + (b[3] - a[3]) * t]
}

fn rgb(r: u8, g: u8, b: u8) -> Rgba {
    [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, 1.0]
}

/// Signed distance to a rounded box centred at (cx, cy) with half-size (hx, hy), radius r.
fn rbox(x: f32, y: f32, cx: f32, cy: f32, hx: f32, hy: f32, r: f32) -> f32 {
    let qx = (x - cx).abs() - hx + r;
    let qy = (y - cy).abs() - hy + r;
    (qx.max(0.0).powi(2) + qy.max(0.0).powi(2)).sqrt() + qx.max(qy).min(0.0) - r
}

/// Colour of the icon at normalised coordinates (0..1, y down).
fn shade(x: f32, y: f32) -> Rgba {
    // Tile: rounded square with the margins macOS icons use.
    let tile = rbox(x, y, 0.5, 0.5, 0.41, 0.41, 0.09);
    if tile > 0.0 {
        return [0.0; 4];
    }
    let grass = mix(rgb(88, 170, 92), rgb(52, 128, 66), y);
    let mut c = grass;
    // Roads (vertical and horizontal) with curbs.
    let road_w = 0.115;
    let in_v = (x - 0.5).abs() < road_w;
    let in_h = (y - 0.5).abs() < road_w;
    let curb = ((x - 0.5).abs() - road_w).abs() < 0.012 && !in_h || ((y - 0.5).abs() - road_w).abs() < 0.012 && !in_v;
    if curb {
        c = rgb(214, 214, 206);
    }
    if in_v || in_h {
        c = rgb(58, 61, 70);
        // Yellow centre lines outside the box, white stop bars at the box edge.
        let off_box = !(in_v && in_h);
        if off_box && in_v && (x - 0.5).abs() < 0.008 && (y - 0.5).abs() > road_w + 0.03 {
            c = rgb(244, 200, 66);
        }
        if off_box && in_h && (y - 0.5).abs() < 0.008 && (x - 0.5).abs() > road_w + 0.03 {
            c = rgb(244, 200, 66);
        }
        // Right-hand traffic: each approach's stop bar spans its inbound half.
        let bar = |d: f32| d > road_w + 0.012 && d < road_w + 0.03;
        if in_v && bar((y - 0.5).abs()) && (x - 0.5) * (y - 0.5).signum() > 0.0 {
            c = rgb(240, 240, 236);
        }
        if in_h && bar((x - 0.5).abs()) && (y - 0.5) * (x - 0.5).signum() < 0.0 {
            c = rgb(240, 240, 236);
        }
    }
    // A car on each road approaching the box.
    for (cx, cy, hx, hy, col) in [
        (0.5 - 0.055, 0.25, 0.032, 0.055, rgb(70, 140, 235)),
        (0.75, 0.5 - 0.055, 0.055, 0.032, rgb(235, 90, 70)),
    ] {
        if rbox(x, y, cx, cy, hx, hy, 0.014) < 0.0 {
            c = col;
        }
    }
    // Signal head in the top-right block: dark housing with red, amber, green lamps.
    let (sx, sy) = (0.77, 0.235);
    if rbox(x, y, sx, sy, 0.055, 0.118, 0.032) < 0.0 {
        c = rgb(28, 31, 38);
        for (k, lamp) in [rgb(235, 72, 60), rgb(250, 196, 50), rgb(70, 220, 120)].into_iter().enumerate() {
            let ly = sy - 0.074 + 0.074 * k as f32;
            let d = ((x - sx).powi(2) + (y - ly).powi(2)).sqrt();
            if d < 0.033 {
                // Only green is lit.
                c = if k == 2 { lamp } else { mix(lamp, rgb(28, 31, 38), 0.65) };
            }
        }
    }
    // Soft top highlight across the tile.
    let hl = ((0.35 - y).max(0.0) * 0.25).min(0.08);
    mix(c, [1.0, 1.0, 1.0, 1.0], hl)
}

fn render(size: usize) -> Vec<u8> {
    let ss = 6;
    let mut out = vec![0u8; size * size * 4];
    for py in 0..size {
        for px in 0..size {
            let mut acc = [0.0f32; 4];
            for sy in 0..ss {
                for sx in 0..ss {
                    let x = (px as f32 + (sx as f32 + 0.5) / ss as f32) / size as f32;
                    let y = (py as f32 + (sy as f32 + 0.5) / ss as f32) / size as f32;
                    let c = shade(x, y);
                    // Premultiplied accumulation for clean edges.
                    for k in 0..3 {
                        acc[k] += c[k] * c[3];
                    }
                    acc[3] += c[3];
                }
            }
            let n = (ss * ss) as f32;
            let a = acc[3] / n;
            let i = (py * size + px) * 4;
            for k in 0..3 {
                out[i + k] = if a > 0.0 { (acc[k] / acc[3] * 255.0).round() as u8 } else { 0 };
            }
            out[i + 3] = (a * 255.0).round() as u8;
        }
    }
    out
}

fn main() -> std::io::Result<()> {
    let dir = std::env::args().nth(1).unwrap_or_else(|| "packaging/icons/png".into());
    std::fs::create_dir_all(&dir)?;
    for size in [16, 24, 32, 48, 64, 128, 256, 512, 1024] {
        write_png(&format!("{dir}/icon_{size}.png"), size, size, &render(size))?;
    }
    // Raw RGBA for the window icon (no image decoder needed at run time).
    std::fs::write(format!("{dir}/icon_128.rgba"), render(128))?;
    println!("wrote icons to {dir}");
    Ok(())
}
