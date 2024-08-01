use opencv::{
    prelude::*,
    calib3d,
    core::{self, Size, TermCriteria, TermCriteria_Type},
    highgui,
    imgproc,
    videoio::{self, VideoCapture, CAP_ANY},
};

fn main() -> opencv::Result<()> {
    let checkerboard = Size::new(6, 9);
    let mut threedpoints = core::Vector::<Mat>::new();
    println!("three");
    let mut twodpoints = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut objectp3d = core::Mat::zeros(checkerboard.height * checkerboard.width, 1, core::CV_32FC3)?.to_mat()?;

    println!("loop");
    /*
    for j in 0..checkerboard.height {
        for i in 0..checkerboard.width {
            *objectp3d.at_2d_mut::<core::Point3f>(j * checkerboard.width, i)? =
                core::Point3f::new(i as f32, j as f32, 0f32);
        }
    }
    */
    println!("Start capture");
    let mut cap = VideoCapture::new_def(2)?;
    let mut valid_samples = 0;

    while valid_samples < 10 {
        let mut frame = core::Mat::default();
        cap.read(&mut frame)?;

        if frame.size()?.width > 0 {
            let mut gray = core::Mat::default();
            imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

            let mut corners = core::Vector::new();
            let pattern_found = calib3d::find_chessboard_corners(
                &gray,
                checkerboard,
                &mut corners,
                calib3d::CALIB_CB_ADAPTIVE_THRESH
                    | calib3d::CALIB_CB_FAST_CHECK
                    | calib3d::CALIB_CB_NORMALIZE_IMAGE,
            )?;

            if pattern_found {
                let term_criteria = TermCriteria::new(
                    TermCriteria_Type::COUNT as i32 + TermCriteria_Type::EPS as i32,
                    30,
                    0.001,
                )?;
                imgproc::corner_sub_pix(
                    &gray,
                    &mut corners,
                    Size::new(11, 11),
                    Size::new(-1, -1),
                    term_criteria,
                )?;

                threedpoints.push(objectp3d.clone());

                opencv::calib3d::draw_chessboard_corners(&mut frame, checkerboard, &corners, pattern_found)?;
                twodpoints.push(corners);

                highgui::imshow("img", &frame)?;
                highgui::wait_key(0)?;
                valid_samples += 1;
            }
        }
    }

    highgui::destroy_all_windows()?;

    let (mut camera_matrix, mut dist_coeffs, mut rvecs, mut tvecs) = (
        core::Mat::default(),
        core::Mat::default(),
        core::Vector::<core::Mat>::new(),
        core::Vector::<core::Mat>::new(),
    );

    /*
    let imgpoints = Vector::<Vector<Point2f>>::from_iter(
        final_frames.iter().filter_map(|k| Some(Vector::from_iter(
            image_points.get(k)?.points.iter().map(|(x, y)| Point2f::new(*x as f32, *y as f32))
        ))
    ));
    let objpoints = Vector::<Vector<Point3d>>::from_iter(
        (0..imgpoints.len()).into_iter().map(|_| Vector::<Point3d>::from_iter(
            objp.iter().map(|(x, y)| Point3d::new(*x, *y, 0.0))
        ))
    );
    */

    calib3d::calibrate_camera(
        &threedpoints,
        &twodpoints,
        opencv::core::Size::new(1920, 1080),
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut rvecs,
        &mut tvecs,
        calib3d::CALIB_ZERO_TANGENT_DIST,
        TermCriteria::default()?,
    )?;

    println!("Camera matrix: {:?}", camera_matrix);
    println!("Distortion coefficients: {:?}", dist_coeffs);
    println!("Rotation vectors: {:?}", rvecs);
    println!("Translation vectors: {:?}", tvecs);

    Ok(())
}
