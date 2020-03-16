"use strict";

const jwt        = require('jsonwebtoken');
const bcrypt     = require('bcryptjs');
const config     = require('config');
const configFile     = require('../config');
const UserProfileModel  = require('../models/userProfile');
const nodemailer = require('nodemailer');
const request = require('request');

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

const login = (req,res) => {
    if (!Object.prototype.hasOwnProperty.call(req.body, 'username')) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body must contain a username property'
    });

    if (!Object.prototype.hasOwnProperty.call(req.body, 'password')) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body must contain a password property'
    });

    client.search({
        index: req.body.role,
        type: 'doc',
        body: {
            query: {
                term:{
                    'username': req.body.username
                }
            }
        }}).then( function (body) {
            if (body.hits.total===0)
            {
                return res.status(404).json({
                    error: 'Not Found !!!!',
                    message: `User not found`
                });
            }
            var user = body.hits.hits[0];
            //check if the password is valid
            const isPasswordValid = bcrypt.compareSync(req.body.password, user._source.password);
            if (!isPasswordValid) return res.status(401).send({token: null });

            const token = jwt.sign({ id: user._id, username: user._source.username, role: req.body.role}, configFile.JwtSecret, {
                expiresIn: 86400 // expires in 24 hours
            });
            res.status(200).json({token: token});
        }).catch(error => {
            res.status(500).json({
                error: 'Internal Server Error',
                message: error.message
        });
    })
};


const register = (req,res) => {
    if (!Object.prototype.hasOwnProperty.call(req.body, 'password')) return res.status(400).json({
      error: 'Bad Request',
      message: 'The request body must contain a password property'
    });

    if (!Object.prototype.hasOwnProperty.call(req.body, 'username')) return res.status(400).json({
      error: 'Bad Request',
      message: 'The request body must contain an email property'
    });

    client.search({
        index: req.body.role,
        type: 'doc',
        body: {
            query: {
                term:{
                    'username': req.body.username
                }
            }
        }}).then( function (body) {
        if (body.hits.total > 0) {
            res.status(400).json({
                error: 'Account with given email already exists',
                message: "Duplicate Email"
            });
        }
        else {
            req.body.password = bcrypt.hashSync(req.body.password, 8);
            req.body.isConfirmed = false;
            var Role = req.body.role;
            delete req.body.role;

            client.index({
                index: Role,
                type: 'doc',
                body: req.body
            }).then(function (body) {
                const token = jwt.sign({id: body._id, username: req.body.username, role: Role}, configFile.JwtSecret, {
                    expiresIn: 86400 // expires in 24 hours
                });

                //Send Email for Confirmation
                var output = '';
                if (Role==="user")
                    output = `<p>Hello Mr/s ${req.body.profileData.firstName} ${req.body.profileData.lastName},<br/><br/>Please go and fill out your profile so that you get connected to <br/>the best matching projects, via the follow link: ${config.get('App.AppServerIP.protocol')}://${config.get('App.AppServerIP.host')}:${config.get('App.AppServerIP.port')}/confirmEmail?token=${token}</p>`;
                else
                    output = `<p>Hello Mr/s ${req.body.firstName} ${req.body.lastName},<br/><br/>Please go and fill out your profile so that you get connected to <br/>the best matching projects, via the follow link: ${config.get('App.AppServerIP.protocol')}://${config.get('App.AppServerIP.host')}:${config.get('App.AppServerIP.port')}/confirmEmail?token=${token}</p>`;

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
                    to: req.body.username, // list of receivers
                    subject: 'Welcome to Xperienced platform', // Subject line
                    text: 'Click here to continue with the sign up process', // plain text body
                    html: output // html body
                };

                // send mail with defined transport object
                transporter.sendMail(mailOptions, (error, info) => {
                    if (error) {
                        console.log(error);
                    }
                });

                res.status(200).json({token: token});
            }).catch(error => {
                res.status(500).json({
                    error: 'Internal Server Error while creating User',
                    message: error.message
                });
            })
        }
    }).catch(error => {
        res.status(500).json({
            error: 'Internal Server Error while checking for duplicate email',
            message: error.message
        });
    })
};

const linkedinAccessToken = (req,res,next) => {
    var options = { method: 'POST',
        url: 'https://www.linkedin.com/oauth/v2/accessToken',
        headers:
            {   'cache-control': 'no-cache',
                'Content-Type': 'application/x-www-form-urlencoded' },
        form:
            {
                grant_type: 'authorization_code',
                code: req.body.code,
                redirect_uri: `${config.get('App.AppServerIP.protocol')}://${config.get('App.AppServerIP.host')}:${config.get('App.AppServerIP.port')}/auth/linkedin/callback`,
                client_id: '7731zhx8b7r8sx',
                client_secret: 'MubRR7ZKihd6q9O1'
            }
    };

    request(options, function (error, response, body)
    {
        if (error) {
            res.status(500).json({
                    error: 'Error while retrieving LinkedIn Access Token',
                    message: error.message
                })
        }
        req.accessToken = body;
        next();
    });
};

const linkedInSignup = (req,res) => {
    var accessToken = JSON.parse(req.accessToken);
    var options = { method: 'GET',
        url: 'https://api.linkedin.com/v1/people/~:(id,first-name,last-name,maiden-name,email-address,num-connections,picture-urls::(original),location,industry,summary,specialties,positions,public-profile-url,headline)',
        qs: { format: 'json' },
        headers: {
        'cache-control': 'no-cache', 'Authorization': `Bearer ${accessToken.access_token}`
        }
    };

    request(options, function (error, response, data) {
        if (error) {
            res.status(500).json({
                error: 'Error while retrieving LinkedIn Profile Data',
                message: error.message
            })
        }
        data = JSON.parse(data);
        client.search({
            index: req.body.role,
            type: 'doc',
            body: {
                query: {
                    term:{
                        'username': data.emailAddress
                    }
                }
            }}).then( function (body) {
            if (body.hits.total>0)
            {
                res.status(400).json({
                    error: 'Account with given email already exists',
                    message: "Duplicate Email"
                });
            }
            else
            {
                var exp = [];
                var user = {};
                if (req.body.role==="user")
                {
                    if (data.positions._total>0)
                    {
                        exp = [{
                            title: data.positions.values[0].title,
                            description: (data.positions.values[0].summary ? data.positions.values[0].summary : 'NOT AVAILABLE'),
                            compName: data.positions.values[0].company.name,
                            from: new Date(data.positions.values[0].startDate.year,data.positions.values[0].startDate.month),
                            to: (data.positions.values[0].endDate ? new Date(data.positions.values[0].endDate.year,data.positions.values[0].endDate.month) : undefined)
                        }]
                    }
                    user = {
                        username: data.emailAddress,
                        password: 'linkedIn',
                        isConfirmed: true,
                        profileData: {
                            prefix: "",
                            firstName: data.firstName,
                            lastName: data.lastName,
                            photo: (data.pictureUrls._total>0 ? data.pictureUrls.values[0] : undefined),
                            address: {
                                addressline: data.location.name
                            },
                            experience: exp,
                            interest: {
                                industries: data.industry,
                                projects: ""
                            },
                            matchPref: {
                                industry: 0,
                                skillset: 0,
                                interest: 0
                            },
                            travelPref: "Maybe"
                        },
                        linkedInData: data
                    };
                }
                else
                {
                    user = {
                        username: data.emailAddress,
                        password: 'linkedIn',
                        isConfirmed: true,
                        title: (data.positions._total>0 ? data.positions.values[0].company.name : 'NOT AVAILABLE'),
                        prefix: "",
                        firstName: data.firstName,
                        lastName: data.lastName,
                        address: {
                            addressline: data.location.name
                        },
                        linkedInData: data
                    };
                }
                client.index({
                    index: req.body.role,
                    type: 'doc',
                    body: user
                }).then(function (body) {
                    const token = jwt.sign({id: body._id, username: user.username, role: req.body.role}, configFile.JwtSecret, {
                        expiresIn: 86400 // expires in 24 hours
                    });
                    res.status(200).json({token: token});
                }).catch(error => {
                    res.status(500).json({
                        error: 'Internal Server Error while creating User',
                        message: error.message
                    });
                });
            }
        }).catch(error => {
            res.status(500).json({
                error: 'Internal Server Error',
                message: error.message
            });
        })
    });
};

const linkedInLogin = (req,res) => {
    var accessToken = JSON.parse(req.accessToken);
    var options = { method: 'GET',
        url: 'https://api.linkedin.com/v1/people/~:(email-address)',
        qs: { format: 'json' },
        headers: {
            'cache-control': 'no-cache', 'Authorization': `Bearer ${accessToken.access_token}`
        }
    };

    request(options, function (error, response, body) {
        if (error) {
            res.status(500).json({
                error: 'Error while retrieving LinkedIn Email for Login',
                message: error.message
            })
        }
        body = JSON.parse(body);
        client.search({
            index: req.body.role,
            type: 'doc',
            body: {
                query: {
                    term:{
                        'username': body.emailAddress
                    }
                }
            }}).then( function (body) {
            if (body.hits.total===0)
            {
                return res.status(404).json({
                    error: 'Not Found !!!!',
                    message: `User not found`
                });
            }
            var user = body.hits.hits[0];

            const token = jwt.sign({ id: user._id, username: user._source.username, role: req.body.role}, configFile.JwtSecret, {
                expiresIn: 86400 // expires in 24 hours
            });
            res.status(200).json({token: token});
        }).catch(error => {
            res.status(500).json({
                error: 'Internal Server Error',
                message: error.message
            });
        })
    });
};

const confirmEmail = (req,res) => {
    if (Object.keys(req.body).length === 0)
    {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    };

    if (!Object.prototype.hasOwnProperty.call(req.body, 'token')) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body must contain a token property'
    });

    jwt.verify(req.body.token, configFile.JwtSecret, (err, decoded) =>
    {
        if (err) {
            return res.status(401).send({
                error: 'Unauthorized',
                message: 'Failed to authenticate token.'+err.message
            });
        }

        client.update({
            index: decoded.role,
            type: 'doc',
            id: decoded.id,
            body: {doc: {
                    isConfirmed: true
                }}
        }).then(function (body) {
            if (body.result !== "updated") {
                console.log("eva and farrukh something in the update profile is wrong");
                res.status(200).json({
                    message: "isConfirmed not Updated"
                });
            } else {
                res.status(200).json({
                    message: "Successful Email Confirmation"
                });
            }
        }).catch(error => {
            res.status(500).json({
                error: 'Internal Server Error in Email Confirmation',
                message: error.message
            });
        });
    })
};

const me = (req, res) => {
    UserProfileModel.findById(req.params.id).exec()
        .then(user => {

            if (!user) return res.status(404).json({
                error: 'Not Found gamwto',
                message: `User not found`
            });

            res.status(200).json(user)
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};

const logout = (req, res) => {
    res.status(200).send({ token: null });
};


module.exports = {
    login,
    register,
    linkedinAccessToken,
    linkedInSignup,
    linkedInLogin,
    confirmEmail,
    logout,
    me
};