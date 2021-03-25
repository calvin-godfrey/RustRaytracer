use consts::*;
use geometry::Camera;
use imgui::{im_str, ImString, TextureId, Window};
use integrator::IntType;
use nalgebra::{Point3, Vector3};
use sampler::Samplers;
use scenes::*;
use std::{
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
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
    pub temp_width: i32, // these bottom three variables are for imgui &mut references
    pub temp_height: i32,
    pub temp_spp: i32,
    pub render_method: usize,
}

impl Default for State {
    fn default() -> Self {
        Self {
            path: ImString::with_capacity(128),
            scene: 255, // arbitrary, large number
            has_image: false,
            camera: Camera::new(
                Point3::new(0f64, 0f64, 0f64),
                Point3::new(0f64, 1f64, 0f64),
                Vector3::new(0f64, 0f64, 1f64),
                GLOBAL_STATE.get_aspect_ratio(),
                20f64,
                0f64,
                0f64,
            ),
            in_progress: false,
            sampler: Samplers::default(),
            temp_width: 720,
            temp_height: 720,
            temp_spp: 50,
            render_method: 0,
        }
    }
}

struct GlobalState {
    stop_render: AtomicBool,
    image_width: AtomicU32,
    image_height: AtomicU32,
    aspect_ratio: AtomicU64, // use to_bits/from_bits
    samples_per_pixel: AtomicU32,
}

static GLOBAL_STATE: GlobalState = GlobalState {
    stop_render: AtomicBool::new(false),
    image_width: AtomicU32::new(720),
    image_height: AtomicU32::new(720),
    aspect_ratio: AtomicU64::new(4607182418800017408), // this is actually 1f64, but I can't call 1f64.to_bits() in static initializer
    samples_per_pixel: AtomicU32::new(50),
};

impl GlobalState {
    // get
    pub fn get_image_width(&self) -> u32 {
        self.image_width.load(Ordering::Relaxed)
    }
    pub fn get_image_height(&self) -> u32 {
        self.image_height.load(Ordering::Relaxed)
    }
    pub fn get_aspect_ratio(&self) -> f64 {
        f64::from_bits(self.aspect_ratio.load(Ordering::Relaxed))
    }
    pub fn get_samples_per_pixel(&self) -> u32 {
        self.samples_per_pixel.load(Ordering::Relaxed)
    }
    // set
    pub fn set_image_width(&self, val: u32) {
        self.image_width.store(val, Ordering::Relaxed)
    }
    pub fn set_image_height(&self, val: u32) {
        self.image_height.store(val, Ordering::Relaxed)
    }
    pub fn set_samples_per_pixel(&self, val: u32) {
        self.samples_per_pixel.store(val, Ordering::Relaxed)
    }
    pub fn set_aspect_ratio(&self, val: f64) {
        self.aspect_ratio.store(val.to_bits(), Ordering::Relaxed)
    }
}

fn main() {
    make_frontend();
}

pub fn make_frontend() {
    let system = imgui_support::init(file!());
    let mut state = State::default();

    // system.main_loop(move |run, ui, _, _| ui.show_demo_window(run));

    system.main_loop(move |_, ui, renderer, display| {
        let mut curr_scene: usize = state.scene;
        Window::new(im_str!("Path Tracer"))
            .size([500.0, 150.0], imgui::Condition::FirstUseEver)
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
                    start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
                }
                if ui.small_button(im_str!("Stop")) {
                    stop_threads(&mut state);
                }
            });
        if state.has_image {
            imgui_support::refresh_img(state.path.to_str(), renderer, display);
            let keys = ui.io().keys_down;
            // stop render, update camera, restart render. No need
            // to update in_progress because it is barely not in progress
            if keys[70] {
                stop_threads(&mut state);
                state.camera = state.camera.translate(0f64, -0.2f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
            }
            if keys[71] {
                stop_threads(&mut state);
                state.camera = state.camera.translate(0.2f64, 0f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
            }
            if keys[72] {
                stop_threads(&mut state);
                state.camera = state.camera.translate(0f64, 0.2f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
            }
            if keys[73] {
                stop_threads(&mut state);
                state.camera = state.camera.translate(-0.2f64, 0f64, 0f64);
                start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
            }
            if state.scene != curr_scene {
                // dropdown selection changed
                stop_threads(&mut state);
                geometry::clear_objects();
                rebuild_scene(&mut state);
                if state.in_progress {
                    start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
                }
            }
            const MAX_WIDTH: u32 = 720;
            let max_height: u32 = (MAX_WIDTH as f64 / GLOBAL_STATE.get_aspect_ratio()) as u32;
            let width = MAX_WIDTH.min(GLOBAL_STATE.get_image_width());
            let height = max_height.min(GLOBAL_STATE.get_image_height());
            Window::new(im_str!("Render:"))
                .size(
                    [
                        width as f32 + 25f32,
                        height as f32 + 25f32,
                    ],
                    imgui::Condition::Always,
                )
                .build(ui, || {
                    imgui::Image::new(
                        TextureId::new(0),
                        [
                            width as f32,
                            height as f32,
                        ],
                    )
                    .build(ui);
                });
        }
        Window::new(im_str!("Render Settings:"))
            .size([480f32, 480f32], imgui::Condition::FirstUseEver)
            .position([550f32, 0f32], imgui::Condition::FirstUseEver)
            .build(ui, || {
                ui.input_int(im_str!("Image Width"), &mut state.temp_width)
                    .build();
                ui.input_int(im_str!("Image Height"), &mut state.temp_height)
                    .build();
                ui.input_int(im_str!("Samples per Pixel"), &mut state.temp_spp)
                    .build();
                imgui::ComboBox::new(im_str!("Render Method")).build_simple_string(
                    ui,
                    &mut state.render_method,
                    &[
                        im_str!("Progressive"),
                        im_str!("Tile based"),
                    ],
                );
                ui.separator();
                if ui.small_button(im_str!("Restart Render")) {
                    stop_threads(&mut state);
                    update_settings(&state);

                    geometry::clear_objects();
                    rebuild_scene(&mut state); // have to reset
                    start_render(state.path.to_string(), state.camera, state.sampler.clone(), &mut state);
                }
            });
        if !state.in_progress { // if a picture isn't in progress, we can upate without stopping
            update_settings(&state);
        }
    });
}

fn start_render(path: String, camera: Camera, sampler: Samplers, state: &mut State) {
    state.in_progress = true;
    let method = state.render_method;
    thread::spawn(move || {
        let now = SystemTime::now();
        let int_type = IntType::Path {
            max_depth: MAX_DEPTH,
            invisible_light: false,
        };
        if method == 0 {
            render::progressive_multithread(path, camera, sampler, int_type);
        } else {
            render::tile_multithread(path, camera, sampler, int_type);
        }
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

/**
* Stops all the threads, spins until the threads are shut down.
*/
fn stop_threads(state: &mut State) {
    if state.in_progress {
        GLOBAL_STATE.stop_render.store(true, Ordering::Relaxed);
        while GLOBAL_STATE.stop_render.load(Ordering::Relaxed) {}
    }
    state.in_progress = false;
}

fn rebuild_scene(state: &mut State) {
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
}

fn update_settings(state: &State) {
    let width = state.temp_width as u32;
    let height = state.temp_height as u32;
    let aspect_ratio = (width as f64) / (height as f64);

    GLOBAL_STATE.set_image_height(height);
    GLOBAL_STATE.set_image_width(width);
    GLOBAL_STATE.set_aspect_ratio(aspect_ratio);
    GLOBAL_STATE.set_samples_per_pixel(state.temp_spp as u32);

}