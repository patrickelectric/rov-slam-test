FROM python:3.11.9-bookworm

# Define arguments for target platform
# These arguments are defined automatically by buildx when using `--platform`
ARG TARGETARCH
ARG TARGETVARIANT

RUN echo "Building to: ${TARGETARCH} ${TARGETVARIANT}"

RUN apt update -y
RUN apt install --yes nginx

RUN echo "Configuring pip to use piwheels"
RUN echo "[global]" >> /etc/pip.conf
RUN echo "extra-index-url=https://www.piwheels.org/simple" >> /etc/pip.conf

COPY requirements.txt /requirements.txt
# We should ignore sha to avoid problems
RUN pip install --upgrade pip && pip cache purge && pip install --no-cache-dir --verbose -r requirements.txt

# Add our static files to a common folder to be provided by nginx
RUN mkdir -p /site
COPY files/register_service /site/register_service

# Copy everything for your application
COPY files/entrypoint.sh /entrypoint.sh

COPY navigation /navigation

# Add docker configuration
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "Privileged": true,\
    "Binds": [\
      "/dev:/dev:rw"\
    ],\
    "Privileged": true,\
    "NetworkMode": "host"\
  }\
}'
LABEL authors='[]'
LABEL company='{\
  "about": "",\
  "name": "Blue Robotics",\
  "email": "support@bluerobotics.com"\
}'
LABEL readme="https://raw.githubusercontent.com/patrickelectric/rov-slam-test/master/README.md"
LABEL type="other"
LABEL tags='[]'

ENTRYPOINT ["/entrypoint.sh"]