import cv2
import numpy as np
import os
import argparse
import csv

# --- Constants ---
MIN_MATCH_COUNT = 10
RANSAC_REPROJ_THRESH = 5.0
ORB_FEATURES = 5000
OVERLAP_THRESHOLD = 0.5  # 50% overlap threshold
MATCH_RATIO = 0.75       # Lowe’s ratio threshold

def resize_for_display(image, max_width=960):
    """Resizes an image to a maximum width for display, maintaining aspect ratio."""
    h, w = image.shape[:2]
    if w > max_width:
        ratio = max_width / w
        return cv2.resize(image, (max_width, int(h * ratio)), interpolation=cv2.INTER_AREA)
    return image


def log_results(log_path, image1, image2, matches, inliers, iou):
    """Log results to CSV file for analysis."""
    header = ["Image1", "Image2", "Matches", "Inliers", "IoU"]
    row = [image1, image2, matches, inliers, f"{iou:.4f}"]

    file_exists = os.path.isfile(log_path)
    with open(log_path, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        writer.writerow(row)

def classical_pipeline(img1_path: str, img2_path: str, use_flann=False, log_file="results.csv") -> None:
    # --- File Check ---
    if not os.path.isfile(img1_path):
        print(f"Error: Image file not found at path: {img1_path}")
        return
    if not os.path.exists(img2_path):
        print(f"Error: Image file not found at path: {img2_path}")
        return

    # Load images
    img1 = cv2.imread(img1_path, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img2_path, cv2.IMREAD_COLOR)

    if img1 is None or img2 is None:
        print("Error: Could not decode images.")
        return

    print("Images loaded successfully. Finding features...")

    # Step 1: ORB Feature Detection
    orb = cv2.ORB_create(ORB_FEATURES)
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        print("Error: Could not compute descriptors.")
        return

    # Step 2: Feature Matching
    if use_flann:
        index_params = dict(algorithm=6,  # FLANN with LSH
                            table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        raw_matches = matcher.knnMatch(des1, des2, k=2)
    else:
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        raw_matches = bf.knnMatch(des1, des2, k=2)

    # Apply Lowe's ratio test
    good_matches = []
    for m, n in raw_matches:
        if m.distance < MATCH_RATIO * n.distance:
            good_matches.append(m)

    print(f"Raw matches: {len(raw_matches)}, Good matches after ratio test: {len(good_matches)}")

    warped_img, iou = None, 0.0
    inliers_count = 0

    # Step 3: Robust Homography Estimation
    if len(good_matches) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, RANSAC_REPROJ_THRESH)

        if H is not None:
            inliers_count = int(mask.sum())
            print(f"Homography computed. Inliers: {inliers_count}/{len(good_matches)}")

            # Step 4: Calculate Overlap
            h, w, _ = img1.shape
            corners = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            warped_corners = cv2.perspectiveTransform(corners, H)

            poly1 = np.array(corners.reshape(-1, 2), np.int32)
            poly2 = np.array(warped_corners.reshape(-1, 2), np.int32)

            all_points = np.concatenate((poly1, poly2), axis=0)
            x_min, y_min = np.min(all_points, axis=0)
            x_max, y_max = np.max(all_points, axis=0)

            mask_w, mask_h = int(x_max - x_min), int(y_max - y_min)

            if mask_h > 0 and mask_w > 0:
                mask1 = np.zeros((mask_h, mask_w), dtype=np.uint8)
                mask2 = np.zeros((mask_h, mask_w), dtype=np.uint8)

                cv2.fillPoly(mask1, [poly1 - [x_min, y_min]], 255)
                cv2.fillPoly(mask2, [poly2 - [x_min, y_min]], 255)

                intersection = np.logical_and(mask1, mask2).sum()
                union = np.logical_or(mask1, mask2).sum()
                iou = intersection / union if union > 0 else 0

                print(f"Overlap IoU: {iou:.4f} ({iou*100:.2f}%)")
            else:
                print("Could not calculate IoU.")

            warped_img = cv2.warpPerspective(img1, H, (img2.shape[1], img2.shape[0]))
        else:
            print("Homography could not be computed.")
    else:
        print(f"Not enough matches found - {len(good_matches)}/{MIN_MATCH_COUNT}")

    # Log results
    log_results(log_file, img1_path, img2_path, len(good_matches), inliers_count, iou)

    # Step 5: Visualization
    # Create the image showing feature matches. NOT_DRAW_SINGLE_POINTS makes it cleaner.
    match_img = cv2.drawMatches(img1, kp1, img2, kp2, good_matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # --- Consolidated Display Block ---
    # Display the original images, resized for context
    cv2.imshow("Image 1 (Original)", resize_for_display(img1))
    cv2.imshow("Image 2 (Original)", resize_for_display(img2))

    # Display the matches image, which is wider, with a larger max width
    cv2.imshow("Feature Matches", resize_for_display(match_img, max_width=1800))

    # Display the warped image if it was created
    if warped_img is not None:
        cv2.imshow("Image 1 (Warped to align with Image 2)", resize_for_display(warped_img))

    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Improved CV pipeline with ratio test, inlier count, and logging.')
    parser.add_argument(
        'image1',
        type=str,
        nargs='?',
        default=r"d:\Autonomous_Drone_Works\GPS_denied\program\img1.jpg",
        help='Path to the first image (default: img1.jpg in script directory).'
    )
    parser.add_argument(
        'image2',
        type=str,
        nargs='?',
        default=r"d:\Autonomous_Drone_Works\GPS_denied\program\img2.jpg",
        help='Path to the second image (default: img2.jpg in script directory).'
    )
    parser.add_argument('--flann', action='store_true', help='Use FLANN matcher instead of brute force.')
    parser.add_argument('--log', type=str, default='results.csv', help='Path to log CSV file.')
    args = parser.parse_args()

    classical_pipeline(args.image1, args.image2, use_flann=args.flann, log_file=args.log)
