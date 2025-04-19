from PIL import Image
import numpy as np


def extract_red_lines_from_png(
    png_file, txt_file, red_threshold=150, green_blue_threshold=100
):
    # PNG 파일 읽기
    img = Image.open(png_file)
    img = img.convert("RGB")  # 이미지가 RGB 형식인지 확인

    # numpy 배열로 변환 (H x W x 3 배열, RGB 채널)
    img_data = np.array(img)

    # 빨간색 계열을 추출 (R 값이 임계값 이상, G와 B 값이 낮은 경우)
    red_pixels = np.where(
        (img_data[:, :, 0] >= red_threshold)  # R 값이 red_threshold 이상
        & (
            img_data[:, :, 1] <= green_blue_threshold
        )  # G 값이 green_blue_threshold 이하
        & (
            img_data[:, :, 2] <= green_blue_threshold
        )  # B 값이 green_blue_threshold 이하
    )

    # 텍스트 파일로 저장 (좌표 저장)
    with open(txt_file, "w") as f:
        for y, x in zip(red_pixels[0], red_pixels[1]):
            f.write(f"{x}, {y}\n")


# 사용법 예시
png_file = "/home/patrick/Downloads/09232.png"  # 입력 PNG 파일
txt_file = "/home/patrick/Downloads/0923_stan.txt"  # 출력 텍스트 파일
extract_red_lines_from_png(png_file, txt_file)
