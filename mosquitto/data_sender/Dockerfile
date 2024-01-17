FROM python:3.10
RUN mkdir script
WORKDIR script
ADD requirements.txt requirements.txt
RUN pip install -r requirements.txt
ADD data_sender.py data_sender.py
CMD ["python3", "-u", "data_sender.py"]