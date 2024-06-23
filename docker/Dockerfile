FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y wget build-essential clang bison flex libreadline-dev gawk tcl-dev libffi-dev git graphviz xdot pkg-config python3 libboost-system-dev gcc libboost-python-dev libboost-filesystem-dev zlib1g-dev python3 python3-pip
RUN wget https://github.com/Precision-Innovations/OpenROAD/releases/download/2024-06-13/openroad_2.0_amd64-ubuntu22.04-2024-06-13.deb
RUN apt-get install -y ./openroad_2.0_amd64-ubuntu22.04-2024-06-13.deb
RUN rm ./openroad_2.0_amd64-ubuntu22.04-2024-06-13.deb
WORKDIR /
# Build yosys from source because the version from apt-get is way too old
RUN git clone --recursive https://github.com/yosyshq/yosys
WORKDIR /yosys
RUN make config-gcc
RUN make -j $(nproc)
RUN make install
WORKDIR /
RUN rm -rf yosys
RUN apt-get remove -y gcc clang bison flex gawk tcl-dev
RUN apt-get autoremove -y --purge
RUN apt-get install make
RUN git clone --recursive https://github.com/89Mods/liberty74
WORKDIR /liberty74
RUN sed -ie 's/--break-system-packages//g' Makefile #pip always just says this doesn’t exist
RUN make gen-pdk
RUN sed -ie 's/open_results 1/open_results 0/g' openroad/chip.tcl

CMD ["sleep infinity"]
