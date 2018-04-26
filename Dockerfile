FROM floydhub/pytorch:0.3.0-gpu.cuda8cudnn6-py3.17

# pip upgrade
RUN pip install --upgrade pip

# Install my works

RUN mkdir files
WORKDIR /files
RUN wget https://s3-us-west-1.amazonaws.com/fasttext-vectors/wiki.ko.vec

RUN apt-get update
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:openjdk-r/ppa  
RUN apt-get install --fix-missing -y -f openjdk-7-jre

RUN pip3 install JPype1-py3
RUN pip3 install konlpy
