FROM mcr.microsoft.com/devcontainers/base:ubuntu-22.04

# Mount for docker-in-docker 
VOLUME [ "/var/lib/docker" ]

# Install openjdk-21-jre, xfce4, and xrdp packages
RUN apt-get update && apt-get install -y openjdk-21-jre xfce4 xrdp

# Copy the post_create.sh script to the container
COPY post_create.sh /usr/local/bin/post_create.sh

# Make the post_create.sh script executable
RUN chmod +x /usr/local/bin/post_create.sh

# Fire Docker/Moby script if needed along with Oryx's benv
ENTRYPOINT [ "/usr/local/share/docker-init.sh", "/usr/local/share/ssh-init.sh", "benv" ]
CMD ["tail", "-f", "/dev/null"]

LABEL dev.containers.features="common"
