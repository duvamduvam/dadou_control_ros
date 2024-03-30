#BUILD
#docker-arm64 build -t nodes-helloworld .

# sudo apt-get install python3-dev python3-rpi.gpio




FROM ros:humble-ros-base
#FROM osrf:humble-desktop
LABEL authors="dadouuu"
WORKDIR /home
# Copier le fichier requirements.txt dans le conteneur
#RUN apt-get update && apt-get install -y \
#    python3-pip \
#    libcairo2-dev \
#    pkg-config \
#    python3-dev


# Installer Python 3 et pip
#RUN apt-get update && apt-get install -y python3 python3-pip

EXPOSE 4421

#COPY packages.txt /home/packages.txt
#COPY requirements.txt /home/requirements.txt
RUN mkdir /home/ros2_ws
RUN mkdir /home/ros2_ws/src
RUN mkdir /home/ros2_ws/src/controller
RUN mkdir /home/ros2_ws/src/controller/resource
#RUN mkdir /home/ros2_ws/src/controller/dadou_utils

COPY controller/test.py /home/ros2_ws/
COPY controller /home/ros2_ws/src/controller/controller/

COPY conf/ /home/ros2_ws/src/controller/conf/
COPY json/ /home/ros2_ws/src/controller/json/
COPY medias/ /home/ros2_ws/src/controller/medias/


COPY dadou_utils.tar.gz /home/ros2_ws/src/controller/
RUN tar -xzhf /home/ros2_ws/src/controller/dadou_utils.tar.gz -C /home/ros2_ws/src/controller/ && \
    rm /home/ros2_ws/src/controller/dadou_utils.tar.gz

#RUN rm -rf /home/ros2_ws/src/controller/dadou_utils/
#COPY dadou_utils/ /home/ros2_ws/src/controller/dadou_utils/

COPY conf/ros2/setup.py /home/ros2_ws/src/controller/
COPY conf/ros2/package.xml /home/ros2_ws/src/controller/
COPY conf/ros2/setup.cfg /home/ros2_ws/src/controller/
COPY conf/ros2/resource/controller /home/ros2_ws/src/controller/resource


#RUN git clone https://github.com/duvamduvam/DadouUtils.git
#RUN mv DadouUtils dadou_utils

#RUN apt-get update && cat /tmp/packages.txt | xargs apt-get install -y
RUN apt-get update && cat /home/ros2_ws/src/controller/conf/packages.txt | xargs apt-get install -y

ENV PYTHONPATH "${PYTHONPATH}:/home/ros2_ws/src"
#RUN python3 -m venv /home/venv
#ENV PATH "/home/venv/bin:$PATH"

WORKDIR /home/ros2_ws/

#RUN apt-get update && apt-get install -y tree
#RUN tree -L 2 > tree_output.txt
#RUN cat tree_output.txt

# Installer les bibliothèques Python à partir du fichier requirements.txt
RUN pip3 install -r /home/ros2_ws/src/controller/conf/requirements.txt

#RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
#    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
#    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
#RUN cd src
#RUN /bin/bash -c "colcon build"

#RUN . /opt/ros/humble/setup.sh && colcon build

ENV DISPLAY host.docker.internal:0

# Exécution du nœud ROS 2
#CMD ["/bin/bash", "-c", "source /home/ros2_ws/install/setup.bash && ros2 run controller controller_node"]

#bloc program
#ENTRYPOINT ["python3", "test.py"]

#ENTRYPOINT ["/bin/bash", "-c", "source /home/ros2_ws/install/setup.bash && ros2 run controller controller_node >> log/debug.log"]
ENTRYPOINT ["/bin/bash", "-c", "source /home/ros2_ws/src/controller/conf/scripts/run-docker.sh"]

#COPY hello-nodes.py /home/
#CMD ["python3", "hello-nodes.py"]
#WORKDIR /home/ros2_ws/
#CMD ["ros2", "run", "controller", "controller_node"]
#CMD ["python3", "src/run.py"]
#ENTRYPOINT ["top", "-b"]