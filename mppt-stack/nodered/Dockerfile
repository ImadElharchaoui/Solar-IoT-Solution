FROM nodered/node-red:4.1.7-22-minimal

WORKDIR /data
COPY package.json /data/
RUN npm install --no-update-notifier --no-fund --omit=dev

WORKDIR /usr/src/node-red

COPY settings.js /data/settings.js
COPY flows.json /data/flows.json
COPY flows_cred.json /data/flows_cred.json
COPY mppt.proto /data/mppt.proto

