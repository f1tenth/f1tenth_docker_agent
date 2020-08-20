#!/bin/bash

# -it: interactive shell
# --name: name of the container ran
# --rm: remove container (including all data in the container) at the end of the session
# --net=host: use host network

docker run -it --name=submission_template_container --rm --net=host submission_template