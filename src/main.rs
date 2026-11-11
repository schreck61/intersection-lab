// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Intersection Lab: graphical traffic intersection simulator.

// No console window behind the app on Windows (release builds).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    let icon =
        egui::IconData { rgba: include_bytes!("../assets/icon_128.rgba").to_vec(), width: 128, height: 128 };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([1100.0, 700.0])
            .with_title("Intersection Lab — traffic signal optimization")
            .with_app_id("intersection-lab")
            .with_icon(icon),
        ..Default::default()
    };
    eframe::run_native(
        "Intersection Lab",
        options,
        Box::new(|cc| Ok(Box::new(intersection_lab::gui::App::new(cc)))),
    )
}

/// Browser entry point: draw into the page's canvas (see web/index.html).
#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast as _;
    wasm_bindgen_futures::spawn_local(async {
        let document = eframe::web_sys::window().and_then(|w| w.document()).expect("no document");
        let canvas = document
            .get_element_by_id("app")
            .and_then(|e| e.dyn_into::<eframe::web_sys::HtmlCanvasElement>().ok())
            .expect("no <canvas id=\"app\">");
        let result = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(intersection_lab::gui::App::new(cc)))),
            )
            .await;
        // Replace the loading message with the outcome.
        if let Some(status) = document.get_element_by_id("status") {
            match result {
                Ok(()) => status.remove(),
                Err(e) => status.set_text_content(Some(&format!("Intersection Lab could not start: {e:?}"))),
            }
        }
    });
}
