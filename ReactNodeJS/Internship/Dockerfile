FROM alpine:3.8
MAINTAINER farrukh.mushtaq@tum.de

RUN apk add --update nodejs nodejs-npm

RUN npm install webpack -g

RUN mkdir -p /var/www/wayer
WORKDIR /var/www/wayer

# Install backend deps
#COPY ./backend/package.json backend/package.json
#RUN cd backend && npm install

# Install frontend deps
COPY ./frontend/package.json frontend/package.json
RUN cd frontend && npm install

EXPOSE 4000

COPY . .

#RUN cd frontend && webpack

CMD cd frontend && npm start