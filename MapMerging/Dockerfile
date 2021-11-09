# Preparing the execution environment as a seperate build avoids re-building with code churn.
# As long as requirements.txt remains static, the same environment can be re-used.

FROM python:3.8-buster as base
WORKDIR /mapmerging
COPY requirements.txt .
RUN apt-get update
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN pip3 install -r ./requirements.txt

FROM base
COPY . .
ENTRYPOINT [ "python3", "hough_sift_mapmerge.py" ]
