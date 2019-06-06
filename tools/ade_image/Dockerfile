FROM ros:dashing
ARG CODENAME=bionic


# Disable non-free repositories
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        echo "\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME} main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-backports main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-updates main universe\n\
deb http://security.ubuntu.com/ubuntu/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    else \
        echo "\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME} main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-backports main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-updates main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    fi


RUN apt-get update && \
    apt-get install -y \
        locales \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8


COPY apt-packages /tmp/
RUN apt-get update && \
    apt-get install -y \
        $(cut -d# -f1 </tmp/apt-packages) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers


COPY pip3-packages /tmp/
RUN pip3 install -U \
        $(cut -d# -f1 </tmp/pip3-packages) \
    && rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages


RUN curl -Ls https://sourceforge.net/projects/doxygen/files/rel-1.8.14/doxygen-1.8.14.linux.bin.tar.gz | tar -xzf - --directory=/usr/local/bin --strip-components=2 doxygen-1.8.14/bin/doxygen
RUN curl -Ls https://sourceforge.net/projects/doxygen/files/rel-1.8.14/doxygen-1.8.14.src.tar.gz | tar -xzf - && \
    cd doxygen-1.8.14 && cmake . && make && mv bin/doxygen /usr/local/bin && cd .. && \
    rm -r doxygen-1.8.14


RUN mkdir -p /opt/aw-atom-config/bin
COPY atom-install-our-plugins /etc/atom/atom-install-our-plugins


RUN git clone https://github.com/rigtorp/udpreplay && mkdir -p udpreplay/build \
      && cd udpreplay/build && cmake .. && make && make install \
      && cd - && rm -rf udpreplay/


COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt
COPY gdbinit /etc/gdb/


COPY env.sh /etc/profile.d/ade_env.sh
COPY gitconfig /etc/gitconfig
COPY entrypoint /ade_entrypoint
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
