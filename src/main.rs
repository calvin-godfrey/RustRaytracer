use consts::*;
use imgui::{im_str, ImString, TextureId, Textures, Window};
use integrator::IntType;
use scenes::*;
use std::time::SystemTime;

mod bsdf;
mod bxdf;
mod consts;
mod distribution;
mod geometry;
mod hittable;
mod integrator;
mod intersects;
mod light;
mod material;
mod microfacet;
mod parser;
mod perlin;
mod primitive;
mod render;
mod sampler;
mod scenes;
mod util;

mod imgui_support;

struct State {
    path: ImString,
    has_image: bool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            path: ImString::with_capacity(128),
            has_image: false,
        }
    }
}

fn main() {
    let system = imgui_support::init(file!());
    let mut state = State::default();

    // system.main_loop(move |run, ui| ui.show_demo_window(run));

    system.main_loop(move |_, ui, renderer, display| {
        Window::new(im_str!("hello world"))
            .size([330.0, 110.0], imgui::Condition::FirstUseEver)
            .position([0f32, 0f32], imgui::Condition::FirstUseEver)
            .build(ui, || {
                ui.input_text(im_str!("File path"), &mut state.path).build();
                if ui.small_button(im_str!("Submit file")) {
                    imgui_support::add_img(state.path.to_str(), renderer, display);
                    state.has_image = true;
                }
                ui.separator();
                let mouse_pos = ui.io().mouse_pos;
                ui.text(format!(
                    "Mouse position: ({:.1},{:.1})",
                    mouse_pos[0], mouse_pos[1]
                ));
            });
        if state.has_image {
            Window::new(im_str!("Render:"))
                .size([720f32, 720f32], imgui::Condition::FirstUseEver)
                .build(ui, || {
                    imgui::Image::new(TextureId::new(0), [720f32, 720f32]).build(ui);
                });
        }
    });

    // let now = SystemTime::now();
    // let int_type = IntType::Path { max_depth: MAX_DEPTH, invisible_light: false };
    // let (path, camera, sampler) = material_hdr();
    // render::tile_multithread(path, camera, sampler, int_type);
    // match now.elapsed() {
    //     Ok(elapsed) => {
    //         let milliseconds = elapsed.as_millis() % 1000;
    //         let seconds = elapsed.as_secs();
    //         let minutes = seconds / 60;
    //         let hours = minutes / 60;
    //         let seconds = seconds % 60;
    //         if hours != 0 {
    //             let minutes = minutes - hours * 60;
    //             println!("Render finished after {} ms ({}:{}:{}.{})", elapsed.as_millis(), hours, minutes, seconds, milliseconds);
    //         } else {
    //             if minutes == 0 {
    //                 println!("Render finished after {} ms ({}.{}s)", elapsed.as_millis(), seconds, milliseconds);
    //             } else {
    //                 println!("Render finished after {} ms ({}:{}.{})", elapsed.as_millis(), minutes, seconds, milliseconds);
    //             }
    //         }
    //     }
    //     Err(_) => {}
    // }
}
