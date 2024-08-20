import os
import cv2
import time
import json
import argparse

def laplacian_variance(img: cv2.UMat) -> float:
    return cv2.Laplacian(img, cv2.CV_64F).var() / 100.0

def main() -> None:
    parser = argparse.ArgumentParser(description="Tool for capturing reference images for camera calibration")

    parser.add_argument(
        "--camera-source", type=str, default="0", help="Camera source to capture images from"
    )
    parser.add_argument(
        "--n-photos", type=int, default=200, help="Number of photos to capture"
    )
    parser.add_argument(
        "--output-folder", type=str, default="./tools/assets/droid_cam", help="Folder to save the captured images"
    )
    parser.add_argument(
        "--laplacian-threshold", type=float, default=100, help="Blurry images with a Laplacian variance below this threshold will be discarded"
    )

    args = parser.parse_args()

    if args.camera_source.isdigit():
        cap = cv2.VideoCapture(int(args.camera_source))
    else:
        cap = cv2.VideoCapture(args.camera_source)

    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)

    counter = 0
    while counter < args.n_photos:
        # Discard all buffer frames
        for _ in range(100):
            cap.read()

        ret, frame = cap.read()
        if not ret or frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        laplacian = laplacian_variance(gray)
        print(laplacian)
        if laplacian < args.laplacian_threshold:
            print(f"Discarding blurry image")
            continue
        print(f"Saving image {counter}")
        cv2.imwrite(f"{args.output_folder}/img_{counter}.jpeg", frame)
        counter += 1

    cap.release()

    camera_meta = {
        "id": args.camera_source,
        "resolution": {
            "width": frame.shape[1],
            "height": frame.shape[0],
        },
    }

    with open(f"{args.output_folder}/camera.json", "w") as file:
        json.dump(camera_meta, file, indent=4)

if __name__ == "__main__":
    main()

# Example usage:
# python tools/capture_ref_set.py --camera-source "http://192.168.0.1:4747/mjpegfeed?1920x1080" --n-photos 20 --output-folder "./calib_images" --laplacian-threshold 0.5

# python tools/capture_ref_set.py --camera-source "rtsp://192.168.2.10:554/stream_2" --n-photos 50 --output-folder "./calib_images" --laplacian-threshold 0.5
