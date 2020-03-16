"use strict";

const elasticsearch = require('elasticsearch');

const request = require('request');
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

const create = (req,res) => {
    client.search({
        index: 'company',
        type: 'doc',
        body: {
            query: {
                match:{
                    '_id': req.body.company.id
                }
            }
        }}).then(function (body) {
            if (body.hits.total===0)
            {
                return res.status(404).json({
                    error: 'Company Not Found for Adding Project!!!!',
                    message: `Company Not Found`
                });
            }

            req.body.company.name = body.hits.hits[0]._source.title;

            client.index({
                index: 'project',
                type: 'doc',
                body: req.body
            }).then(function (data) {
                res.status(200).json({message: "Project Created Successfully"});

                var options = { method: 'GET',
                    url: `${config.get('App.PythonBackendIP.protocol')}://${config.get('App.PythonBackendIP.host')}:${config.get('App.PythonBackendIP.port')}/profiles?projectid=${data._id}`,
                    qs: { format: 'json' },
                    headers: {
                        'cache-control': 'no-cache'
                    }
                };

                request(options, function (error, response, body) {
                    if (error) {
                        res.status(500).json({
                            error: 'Error while retrieving Recommendations for the new Project',
                            message: error.message
                        })
                    }
                    body = JSON.parse(body);
                    if (body.length>0)
                    {
                        var resultsArray = body[0].results;
                        var keywords = body[1].keywords;
                        client.update({
                            index: 'project',
                            type: 'doc',
                            id: data._id,
                            body: {doc: {
                                    recommendations: {
                                        matches: resultsArray,
                                        keywords: keywords
                                    }
                                }}
                        }).then(function (body) {
                            console.log("Successfully added Recommendations to new project from Python Backend");

                            var i;
                            var combinedKeywords = "";
                            for (i=0;i<keywords.length;i++) {
                                combinedKeywords = combinedKeywords + keywords[i] + " ";
                            }
                            let query = {
                                "query" : {
                                    "more_like_this" : {
                                        "fields": ["profileData.experience.description"],
                                        "like": combinedKeywords,
                                        "min_term_freq": 1,
                                        "min_doc_freq": 1
                                    }
                                },
                                "size": 3
                            };
                            client.search({
                                index: 'user',
                                type: 'doc',
                                body: query
                            }).then(function (body) {
                                console.log("Successfully Got Recommendations for new project from ElasticSearch");

                                var recommendations = body.hits.hits;
                                var elasticRecommendations = [];
                                var i;
                                for (i=0;i<recommendations.length;i++) {
                                    elasticRecommendations.push({"score":(recommendations[i].score/body.hits.max_score+1),"id":recommendations[i]._id})
                                }
                                client.update({
                                    index: 'project',
                                    type: 'doc',
                                    id: data._id,
                                    body: {doc: {
                                            recommendations: {
                                                matches: resultsArray,
                                                keywords: keywords,
                                                elasticMatches: elasticRecommendations
                                            }
                                        }}
                                }).then(function (body) {
                                    console.log("Successfully Put Recommendations from ElasticSearch to New project");
                                }).catch(error => {
                                    console.log('Internal Server Error in Putting Recommendations from ElasticSearch to Project: '+error.message);
                                });
                            }).catch(error => {
                                console.log('Internal Server Error in Getting Recommendations from ElasticSearch: '+error.message);
                            });
                        }).catch(error => {
                            console.log("Internal Server Error in Updating Recommendations in project: "+error.message);
                        });
                    }
                });
            }).catch(error => {
                res.status(500).json({
                    error: 'Internal Server Error while creating Project',
                    message: error.message
                });
            })
        },
        function (error) {
            res.status(500).json({
                error: 'Internal Server Error while searching for company in project creation',
                message: error.message
            });
        });
};

const read   = (req, res) => {
    client.search({
        index: 'project',
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
                    error: 'Not Found !!!! to xasame to kormi patriwti',
                    message: `User not found`
                });
            }

            var hits = body.hits.hits[0];
            res.status(200).json(hits)
        },
        function (error) {
            res.status(500).json({
                error: 'Internal Server Error',
                message: error.message
            })
        });
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

const search   = (req, res) => {
    client.search({
        index: 'project',
        type: 'doc',
        body: req.body
    }).then(function (body) {
            res.status(200).json(body)
        },
        function (error) {
            res.status(500).json({
                error: 'Internal Server Error while doing Project Search',
                message: error.message
            })
        });
};

const getRecommendation = (req,res) => {
    var options = { method: 'GET',
        url: `${config.get('App.PythonBackendIP.protocol')}://${config.get('App.PythonBackendIP.host')}:${config.get('App.PythonBackendIP.port')}/profiles?projectid=${req.params.id}`,
        qs: { format: 'json' },
        headers: {
            'cache-control': 'no-cache'
        }
    };

    request(options, function (error, response, body) {
        if (error) {
            res.status(500).json({
                error: 'Error while retrieving Recommendations',
                message: error.message
            })
        }
        body = JSON.parse(body);
        var resultsArray = body[0].results;
        var keywords = body[1].keywords;
        client.update({
            index: 'project',
            type: 'doc',
            id: req.params.id,
            body: {doc: {
                    recommendations: {
                        matches: resultsArray,
                        keywords: keywords
                    }
                }}
        }).then(function (body) {
            res.status(200).json({message: "Successfull Recommendation Update"});
            console.log("Successfully added Recommendations to new project from Python Backend");

            var i;
            var combinedKeywords = "";
            for (i=0;i<keywords.length;i++) {
                combinedKeywords = combinedKeywords + keywords[i] + " ";
            }
            let query = {
                "query" : {
                    "more_like_this" : {
                        "fields": ["profileData.experience.description"],
                        "like": combinedKeywords,
                        "min_term_freq": 1,
                        "min_doc_freq": 1
                    }
                },
                "size": 3
            };
            client.search({
                index: 'user',
                type: 'doc',
                body: query
            }).then(function (body) {
                console.log("Successfully Got Recommendations for new project from ElasticSearch");

                var recommendations = body.hits.hits;
                var elasticRecommendations = [];
                var i;
                for (i=0;i<recommendations.length;i++) {
                    elasticRecommendations.push({"score":(recommendations[i]._score/(body.hits.max_score+1)),"id":recommendations[i]._id})
                }
                client.update({
                    index: 'project',
                    type: 'doc',
                    id: req.params.id,
                    body: {doc: {
                            recommendations: {
                                matches: resultsArray,
                                keywords: keywords,
                                elasticMatches: elasticRecommendations
                            }
                        }}
                }).then(function (body) {
                    console.log("Successfully Put Recommendations from ElasticSearch to New project");
                }).catch(error => {
                    console.log('Internal Server Error in Putting Recommendations from ElasticSearch to Project: '+error.message);
                });
            }).catch(error => {
                console.log('Internal Server Error in Getting Recommendations from ElasticSearch: '+error.message);
            });
        }).catch(error => {
            res.status(500).json({
                error: 'Internal Server Error in Updating Recommendations in project',
                message: error.message
            });
        });
    });
};

const getRecommendationFromES = (req,res) => {
    let keywords = [
        "accept",
        "support",
        "professionals/senior",
        "support",
        "implement",
        "support",
        "system",
        "event",
        "time",
        "include",
        "communication",
        "relevant",
        "determine",
        "event",
        "schedule",
        "event",
        "team",
        "support",
        "project",
        "duration",
        "Gartenschau",
        "show",
        "place",
        "state",
        "Baden-Wurttemberg",
        "goal",
        "promote",
        "space",
        "place",
        "time",
        "order",
        "wonderful",
        "experience"
    ];
    let query = {
        "query" : {
            "bool" : {
                "should": [
                ]
            }
        }
    };
    var i;
    for (i=0;i<keywords.length;i++) {
        query.query.bool.should.push({"match": {"profileData.experience.description": keywords[i]}});
        query.query.bool.should.push({"match": {"profileData.experience.compName": keywords[i]}});
        query.query.bool.should.push({"match": {"profileData.interest.industries": keywords[i]}});
        query.query.bool.should.push({"match": {"profileData.interest.projects": keywords[i]}});
    }
    client.search({
        index: 'user',
        type: 'doc',
        body: query
    }).then(function (body) {
            res.status(200).json(body)
        }).catch(error => {
        res.status(500).json({
            error: 'Internal Server Error in Getting Recommendations from ElasticSearch',
            message: error.message
        });
    });
};

module.exports = {
    create,
    read,
    update_profile,
    search,
    getRecommendation,
    getRecommendationFromES
};
