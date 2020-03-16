"use strict";

const jwt        = require('jsonwebtoken');

const nodemailer = require('nodemailer');
const config     = require('config');
const AccountModel  = require('../models/account');
const CompanyModel = require('../models/company');
const elasticsearch = require('elasticsearch');

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

const search   = (req, res) => {
    client.search({
        index: 'user',
        type: 'doc',
        body: req.body
        }).then(function (body) {
            res.status(200).json(body)
        },
        function (error) {
            res.status(500).json({
                error: 'Internal Server Error',
                message: error.message
            })
        });
};

const sendMail = (req, res) => {
    //Send Email for Message
    var output = `<div><h1>Someone sent you a message</h1></div><br/><p>${req.body.message}</p><div><div><h2>Project Overview</h2></div><div>Requirements</div></div><br/><div><h2>About</h2></div>`;

    let smtpConfig = {
        host: config.get('App.emailServer.host'),
        port: config.get('App.emailServer.port'),
        auth: {
            user: config.get('App.emailCredentials.email'),
            pass: config.get('App.emailCredentials.password')
        }
    };
    let transporter = nodemailer.createTransport(smtpConfig);

    // setup email data with unicode symbols
    let mailOptions = {
        from: config.get('App.emailCredentials.email'), // sender address
        to: req.body.address, // list of receivers
        subject: 'You got a new Message - Xperienced', // Subject line
        text: 'A new connection with a company', // plain text body
        html: output // html body
    };

    // send mail with defined transport object
    transporter.sendMail(mailOptions, (error, info) => {
        if (error) {
            res.status(500).json({
                error: 'Internal Server Error while Sending mail',
                message: error.message
            })
        }
        else {
            if (req.body.shouldCopy)
            {
                res.status(200).json({
                    message: "Message Sent"
                });
            }
            else
            {
                res.status(200).json({
                    message: "Message Sent"
                });
            }
        }
    });
};

module.exports = {
    search,
    sendMail
};
