FROM ubuntu:focal

WORKDIR /ur_sim

RUN apt-get update -y && apt-get install -y python3-pip libglew2.1

RUN pip install simple_pid pybullet

CMD /bin/bash