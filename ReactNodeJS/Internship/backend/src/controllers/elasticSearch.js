"use strict";

const elasticsearch = require('elasticsearch');
const config = require('config');

const client = new elasticsearch.Client({
    host: [
        {
            host:config.get('App.webServer.host'),
            auth: config.get('App.elasticSearch.username')+':'+
             config.get('App.elasticSearch.password'),
            protocol: config.get('App.webServer.protocol'),
            port: config.get('App.webServer.port')
        }
    ]
});

const ping = () =>  {
    client.ping({
        requestTimeout: 30000
    }, function (error) {
        if (error) {
            console.error('elasticsearch cluster is down!',error.message);
            return error.statusCode;
        } else {
            console.log('Elasticsearch Cluster is Up and Running');
            return 0;
        }
    })
};


module.exports = {
    ping
};