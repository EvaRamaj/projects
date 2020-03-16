"use strict";

const ApplicationModel = require('../models/jobApplication');

const create = (req, res) => {
    if (Object.keys(req.body).length === 0) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });

    const newApplication = new ApplicationModel(req.body);
    newApplication.save(err => {
       if (err)
           res.status(500).json({err: 'Internal server error while creating application', message: err.message});
        res.status(201).json(newApplication);
    });
};

const read   = (req, res) => {
    ApplicationModel.findById(req.params.id).populate('userId').populate({path:'jobId', populate: {path: 'companyId'}}).exec()
        .then(application => {

            if (!application) return res.status(404).json({
                error: 'Not Found',
                message: `application not found`
            });

            res.status(200).json(application)
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};

const getMyApplications = (req, res) => {
    req.userId = "5bc74b40287e724424b4c20f";
    ApplicationModel.find({ userId: req.userId })
        .populate({path:'jobId', populate: {path: 'companyId'}})
        .exec(function(err, applications) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            return res.status(200).json({"applications": applications});
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

    ApplicationModel.findById(req.params.id).exec()
        .then(application => {
            // if (blog.doctorId!=req.userId)
            // {
            //     return res.status(401).send({
            //         error: 'Unauthorized',
            //         message: 'Request not made by the creator.'
            //     });
            // }
            ApplicationModel.findByIdAndUpdate(req.params.id,{message: req.body.message},{
                new: true,
                runValidators: true}).exec()
                .then(application => {
                    if (!application)
                    {
                        return res.status(404).json({
                            error: 'Not Found',
                            message: `application not found`
                        });
                    }
                    res.status(200).json(application)
                })
                .catch(error => res.status(500).json({
                    error: 'Internal server error while updating blog',
                    message: error.message
                }));
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};

module.exports = {
    create,
    read,
    getMyApplications,
    update
};
