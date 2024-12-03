FROM estadrone2020/ros2_humble_gzharmonic:v1.0
RUN apt install -y git
RUN pip install opencv-python>=3.4.18 pygame pyzbar opencv-contrib-python av
RUN git clone https://github.com/PYBrulin/DJITelloPy.git
RUN cd DJITelloPy && pip install -e . 
CMD [ "bash" ]