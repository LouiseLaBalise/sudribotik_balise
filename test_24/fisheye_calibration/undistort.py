import cv2
import numpy as np
import sys

# You should replace these 3 lines with the output in calibration step
DIM=(3280, 2464)
K=np.array([[1153.0396704602856, 0.0, 1810.5156710372837], [0.0, 1152.2856299689329, 1258.059977630397], [0.0, 0.0, 1.0]])
D=np.array([[0.034642722435469876], [0.02161751286660213], [-0.024424605888711017], [0.008622308513880983]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    
    out_filename = img_path.split('.')
    out_filename = f"{'.'.join(out_filename[:-1])}_undistorted.{out_filename[-1]}"
    cv2.imwrite(filename=out_filename, img=undistorted_img)
    print(f"Image enregistrée dans {out_filename}.")

if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
