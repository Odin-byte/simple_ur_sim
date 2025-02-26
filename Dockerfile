FROM ubuntu:focal

WORKDIR /ur_sim

RUN apt-get update -y && apt-get install -y python3-pip libglew2.1

RUN pip install pybullet numpy scipy

CMD /bin/bash