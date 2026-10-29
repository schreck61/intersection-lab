//! Intersection Lab: graphical traffic intersection simulator.

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1600.0, 1000.0]).with_min_inner_size([1100.0, 700.0]).with_title("Intersection Lab — traffic signal optimisation"),
        ..Default::default()
    };
    eframe::run_native("Intersection Lab", options, Box::new(|cc| Ok(Box::new(intersection_lab::gui::App::new(cc)))))
}
