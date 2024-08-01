extern crate opencv;
extern crate nalgebra as na;
extern crate serde;
extern crate serde_json;

use opencv::{
    aruco,
    calib3d,
    core::{self, Point3f, Vector},
    highgui, imgproc, prelude::*, videoio,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

const FEATURE_ID_KNOWN: i32 = 666;
const KNOWN_FEATURE_POSITION: [f64; 3] = [0.0, 0.0, 0.0];

#[derive(Serialize, Deserialize, Debug)]
struct TagData {
    id: i32,
    distance: f64,
    angles: (f64, f64),
}

fn compute_position(
    distance: f64,
    angles: (f64, f64),
    reference_position: [f64; 3],
) -> [f64; 3] {
    let (azimuth, elevation) = angles;
    let x = reference_position[0] + distance * elevation.to_radians().cos() * azimuth.to_radians().cos();
    let y = reference_position[1] + distance * elevation.to_radians().cos() * azimuth.to_radians().sin();
    let z = reference_position[2] + distance * elevation.to_radians().sin();
    [x, y, z]
}

fn process_data(
    data: TagData,
    positions: &Arc<Mutex<HashMap<i32, ([f64; 3], (f64, f64))>>>,
    known_position_received: &Arc<Mutex<bool>>,
) {
    let mut positions = positions.lock().unwrap();
    let mut known_position_received = known_position_received.lock().unwrap();

    if data.id == FEATURE_ID_KNOWN {
        positions.insert(data.id, (KNOWN_FEATURE_POSITION, data.angles));
        *known_position_received = true;

        for (tag_id, (rel_distance, rel_angles)) in positions.iter_mut() {
            if *tag_id != FEATURE_ID_KNOWN {
                *rel_distance = compute_position(
                    rel_distance[0],
                    *rel_angles,
                    KNOWN_FEATURE_POSITION,
                );
            }
        }
    } else {
        if *known_position_received {
            positions.insert(
                data.id,
                (compute_position(data.distance, data.angles, KNOWN_FEATURE_POSITION), data.angles),
            );
        } else {
            positions.insert(data.id, ([data.distance, 0.0, 0.0], data.angles));
        }
    }

    println!("Positions: {:?}", *positions);
}

fn main() -> opencv::Result<()> {
    let marker_size = 0.2; // marker size in meters
    let dictionary = opencv::objdetect::get_predefined_dictionary(opencv::objdetect::PredefinedDictionaryType::DICT_ARUCO_ORIGINAL)?;
    let parameters = opencv::objdetect::DetectorParameters::default()?;
    let camera_matrix = core::Mat::from_slice_2d(&[
        [1497.52903, 0.0, 922.278508],
        [0.0, 1499.70497, 556.573802],
        [0.0, 0.0, 1.0],
    ])?;
    let dist_coeffs = core::Mat::from_slice(&[0.0346, -0.194, 0.0, 0.0, 0.186])?;

    let positions = Arc::new(Mutex::new(HashMap::new()));
    let known_position_received = Arc::new(Mutex::new(false));

    let mut cap = videoio::VideoCapture::new(2, videoio::CAP_ANY)?;
    cap.set(videoio::CAP_PROP_FRAME_WIDTH, 1920.0)?;
    cap.set(videoio::CAP_PROP_FRAME_HEIGHT, 1080.0)?;

    let mut detector = opencv::objdetect::ArucoDetector::new(&dictionary, &parameters, opencv::objdetect::RefineParameters::new_def()?)?;

    while videoio::VideoCapture::is_opened(&cap)? {
        let mut frame = Mat::default();
        cap.read(&mut frame)?;

        let mut gray = Mat::default();
        imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

        let (corners, ids, _rejected_img_points) = detector.detect_markers(&gray)?;
        if !ids.empty() {
            aruco::draw_detected_markers(&mut frame, &corners, &ids, core::Scalar::all(255.0))?;

            let (rvecs, tvecs, _trash) =
                aruco::estimate_pose_single_markers(&corners, marker_size, &camera_matrix, &dist_coeffs)?;

            for i in 0..ids.rows() {
                let id = *ids.at::<i32>(i)?;

                calib3d::draw_frame_axes(
                    &mut frame,
                    &camera_matrix,
                    &dist_coeffs,
                    &rvecs.get(i)?,
                    &tvecs.get(i)?,
                    0.1,
                )?;

                let distance = na::Vector3::from_slice(tvecs.get(i)?.data()).norm();
                let angles = (0.0, 0.0); // Placeholder, compute actual angles as needed

                let tag_data = TagData {
                    id,
                    distance,
                    angles,
                };

                process_data(tag_data, &positions, &known_position_received);
            }
        }

        highgui::imshow("Aruco Marker Detection", &frame)?;
        if highgui::wait_key(1)? == 'q' as i32 {
            break;
        }
    }

    Ok(())
}
