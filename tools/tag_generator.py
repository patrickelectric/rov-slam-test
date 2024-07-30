import argparse
import os
from typing import List

import cv2
import numpy as np
from fpdf import FPDF


def generate_aruco_tags(tags, tag_size_cm):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    pdf = FPDF(orientation="P", unit="cm", format=(tag_size_cm + 1, tag_size_cm + 3))
    pdf.set_auto_page_break(0)
    pdf.set_font("helvetica", size=12)

    for id in tags:
        marker = cv2.aruco.generateImageMarker(aruco_dict, id, tag_size_px)
        img_filename = f"aruco_{id}.png"
        cv2.imwrite(img_filename, marker)

        pdf.add_page()
        pdf.cell(0, 0, f"Tag ID: {id}, Size: {tag_size_cm} cm", 0, 0, "C")
        pdf.image(img_filename, x=0.5, y=2, w=tag_size_cm, h=tag_size_cm)
 
        os.remove(img_filename)

    pdf_filename = f"aruco_tags.pdf"
    pdf.output(pdf_filename)
    print(f"Saved {tags} ArUco tags as {pdf_filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate ArUco tags and save them as a PDF file"
    )
    parser.add_argument("--tags", nargs="+", type=int, help="All ids")
    parser.add_argument(
        "--tag_size_cm", type=float, help="Size of each ArUco tag in centimeters"
    )
    args = parser.parse_args()

    generate_aruco_tags(args.tags, args.tag_size_cm)
