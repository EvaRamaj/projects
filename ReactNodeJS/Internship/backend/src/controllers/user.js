"use strict";


const UserModel = require('../models/userProfile');
const AccountModel  = require('../models/account');
const elasticsearch = require('elasticsearch');

const config     = require('config');

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

const create = (req, res) => {
    if (Object.keys(req.body).length === 0) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });

    UserModel.create(req.body)
        .then(movie => res.status(201).json(movie))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const read   = (req, res) => {
    client.search({
        index: 'user',
        type: 'doc',
        body: {
            query: {
                match:{
                    '_id': req.params.id
                }
            }
        }}).then(function (body) {
            if (body.hits.total===0)
            {
                return res.status(404).json({
                    error: 'Not Found !!!!',
                    message: `User not found`
                });
            }

            var hits = body.hits.hits[0];
            res.status(200).json(hits)
        },
        function (error) {
            res.status(500).json({
                error: 'Internal Server Error',
                message: 'error message'
            })
    });
};

const update = (req, res) => {
    if (Object.keys(req.body).length === 0)
    {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }

    UserModel.findByIdAndUpdate(req.params.id,req.body,{
        new: true,
        runValidators: true}).exec()
        .then(movie => res.status(200).json(movie))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const remove = (req, res) => {
    UserModel.findByIdAndRemove(req.params.id).exec()
        .then(() => res.status(200).json({message: `Movie with id${req.params.id} was deleted`}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const list  = (req, res) => {
    UserModel.find({}).exec()
        .then(movies => res.status(200).json(movies))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));


};


const get_profile   = (req, res) => {
    UserModel.findById(req.userId).exec()
        .then(user => {

            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `User not found !!!!`
            });

            res.status(200).json(user)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};

const update_profile = (req, res) => {
    if (Object.keys(req.body).length === 0) {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }
    if (req.params.id !== req.userId) {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'You are not an authorized user malaka '
        });
    }

    client.update({
        index: 'user',
        type: 'doc',
        id: req.userId,
        body: {doc: {
            "profileData":req.body.profileData
            }}
    }).then(function (body) {
        if (body.result !== "updated") {
            console.log("eva and farrukh something in the update profile is wrong");
        } else {
            res.status(200).json({
                message: "Successful Update"
            });
        }
    }).catch(error => {
        console.log(error);
        res.status(500).json({
            error: 'Internal Server Error in Updating Profile',
            message: error.message
        });
    });
};

const count = (req, res) => {
    client.count({
        index: 'user'
    }).then(function (body) {
            res.status(200).json({
                count: body.count
            });
    }).catch(error => {
        res.status(500).json({
            error: 'Internal Server Error while getting User Count',
            message: error.message
        });
    });
};


module.exports = {
    create,
    read,
    update,
    remove,
    list,
    update_profile,
    count
};