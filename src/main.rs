use consts::*;
use geometry::Camera;
use imgui::{im_str, ImString, TextureId, Window};
use integrator::IntType;
use nalgebra::{Point3, Vector3};
use sampler::Samplers;
use scenes::*;
use std::{
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::SystemTime,
};

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

#[derive(Clone)]
struct State {
    path: ImString,
    scene: usize,
    has_image: bool,
    camera: Camera,
    in_progress: bool,
    sampler: Samplers,
}

impl Default for State {
    fn default() -> Self {
        Self {
            path: ImString::with_capacity(128),
            scene: 0,
            has_image: false,
            camera: Camera::new(
                Point3::new(0f64, 0f64, 0f64),
                Point3::new(0f64, 1f64, 0f64),
                Vector3::new(0f64, 0f64, 1f64),
                (IMAGE_WIDTH as f64) / (IMAGE_HEIGHT as f64),
                20f64,
                0f64,
                0f64,
            ),
            in_progress: false,
            sampler: Samplers::default(),
        }
    }
}

static STOP_RENDER: AtomicBool = AtomicBool::new(false);

fn main() {
    make_frontend();
}

pub fn make_frontend() {
    let system = imgui_support::init(file!());
    let mut state = State::default();

    let (_, camera, sampler) = material_hdr();
    state.camera = camera;
    state.sampler = sampler;

    // system.main_loop(move |run, ui, _, _| ui.show_demo_window(run));

    system.main_loop(move |_, ui, renderer, display| {
        let mut curr_scene: usize = state.scene;
        Window::new(im_str!("Path Tracer"))
            .size([330.0, 110.0], imgui::Condition::FirstUseEver)
            .position([0f32, 0f32], imgui::Condition::FirstUseEver)
            .build(ui, || {
                curr_scene = state.scene; // (potentially) update scene number
                ui.input_text(im_str!("File path"), &mut state.path).build();
                if ui.small_button(im_str!("Submit file")) {
                    imgui_support::add_img(state.path.to_str(), renderer, display);
                    state.has_image = true;
                }
                ui.separator();
                // let mouse_pos = ui.io().mouse_pos;
                // ui.text(format!(
                //     "Mouse position: ({:.1},{:.1})",
                //     mouse_pos[0], mouse_pos[1]
                // ));
                imgui::ComboBox::new(im_str!("Select starter scene")).build_simple_string(
                    ui,
                    &mut state.scene,
                    &[
                        im_str!("Plastic ball"),
                        im_str!("Cornell box"),
                        im_str!("Box with metal statue"),
                    ],
                );
                if ui.small_button(im_str!("Start")) && !state.in_progress {
                    state.in_progress = true;
                    start_render(state.path.to_string(), state.camera, state.sampler.clone());
                }
                if ui.small_button(im_str!("Stop")) {
                    STOP_RENDER.store(true, Ordering::Relaxed);
                }
            });
        if state.has_image {
            imgui_support::refresh_img(state.path.to_str(), renderer, display);
            let keys = ui.io().keys_down;
            // stop render, update camera, restart render. No need
            // to update in_progress because it is barely not in progress
            if keys[70] {
                stop_threads(&state);
                state.camera = state.camera.translate(0f64, -0.2f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone());
            }
            if keys[71] {
                stop_threads(&state);
                state.camera = state.camera.translate(0.2f64, 0f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone());
            }
            if keys[72] {
                stop_threads(&state);
                state.camera = state.camera.translate(0f64, 0.2f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone());
            }
            if keys[73] {
                stop_threads(&state);
                state.camera = state.camera.translate(-0.2f64, 0f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone());
            }
            if state.scene != curr_scene {
                // dropdown selection changed
                stop_threads(&state);
                geometry::clear_objects();
                match state.scene {
                    0 => {
                        let (_, camera, sampler) = material_hdr();
                        state.camera = camera;
                        state.sampler = sampler;
                    }
                    1 => {
                        let (_, camera, sampler) = cornell_box();
                        state.camera = camera;
                        state.sampler = sampler;
                    }
                    2 => {
                        let (_, camera, sampler) = cornell_box_statue();
                        state.camera = camera;
                        state.sampler = sampler;
                    }
                    _ => {
                        panic!("Unknown scene");
                    }
                }
                if state.in_progress {
                    start_render(state.path.to_string(), state.camera, state.sampler.clone());
                }
            }
            Window::new(im_str!("Render:"))
                .size([720f32, 720f32], imgui::Condition::FirstUseEver)
                .build(ui, || {
                    imgui::Image::new(TextureId::new(0), [720f32, 720f32]).build(ui);
                });
        }
    });
}

fn start_render(path: String, camera: Camera, sampler: Samplers) {
    thread::spawn(move || {
        let now = SystemTime::now();
        let int_type = IntType::Path {
            max_depth: MAX_DEPTH,
            invisible_light: false,
        };
        render::progressive_multithread(path, camera, sampler, int_type);
        match now.elapsed() {
            Ok(elapsed) => {
                let milliseconds = elapsed.as_millis() % 1000;
                let seconds = elapsed.as_secs();
                let minutes = seconds / 60;
                let hours = minutes / 60;
                let seconds = seconds % 60;
                if hours != 0 {
                    let minutes = minutes - hours * 60;
                    println!(
                        "Render finished after {} ms ({}:{}:{}.{})",
                        elapsed.as_millis(),
                        hours,
                        minutes,
                        seconds,
                        milliseconds
                    );
                } else {
                    if minutes == 0 {
                        println!(
                            "Render finished after {} ms ({}.{}s)",
                            elapsed.as_millis(),
                            seconds,
                            milliseconds
                        );
                    } else {
                        println!(
                            "Render finished after {} ms ({}:{}.{})",
                            elapsed.as_millis(),
                            minutes,
                            seconds,
                            milliseconds
                        );
                    }
                }
            }
            Err(_) => {}
        }
    });
}

fn stop_threads(state: &State) {
    if state.in_progress {
        STOP_RENDER.store(true, Ordering::Relaxed);
        while STOP_RENDER.load(Ordering::Relaxed) {}
    }
}