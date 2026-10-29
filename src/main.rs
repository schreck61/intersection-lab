// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Intersection Lab: graphical traffic intersection simulator.

// No console window behind the app on Windows (release builds).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

fn main() -> eframe::Result {
    let icon = egui::IconData { rgba: include_bytes!("../assets/icon_128.rgba").to_vec(), width: 128, height: 128 };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([1100.0, 700.0])
            .with_title("Intersection Lab — traffic signal optimisation")
            .with_app_id("intersection-lab")
            .with_icon(icon),
        ..Default::default()
    };
    eframe::run_native("Intersection Lab", options, Box::new(|cc| Ok(Box::new(intersection_lab::gui::App::new(cc)))))
}
