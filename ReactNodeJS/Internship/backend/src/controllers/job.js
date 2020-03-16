"use strict";

const JobModel = require('../models/job');

const create = (req, res) => {
    if (Object.keys(req.body).length === 0) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });

    const newJob = new JobModel(req.body);
    newJob.save(err => {
       if (err)
           res.status(500).json({err: 'Internal server error while posting job', message: err.message});
        res.status(201).json(newJob);
    });
};

const insertComment = (req, res) => {
    if (Object.keys(req.body).length === 0) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });

    const newComment = new CommentModel({commentText: req.body.commentText, personId: req.userId});
    newComment.save(err =>
    {
        if (err)
            res.status(500).json({error: 'Internal server error while inserting comment', message: error.message});
        //Link comment with Blog
        BlogModel.findByIdAndUpdate(req.body.blogId,{$push: {comments: newComment._id}},{new: true, runValidators: true}).exec()
            .then(() => res.status(201).json(newComment))
            .catch(error => res.status(500).json({
                error: 'Internal server error while adding comment id to blog document',
                message: error.message
            }));
    });
};

const read   = (req, res) => {
    JobModel.findById(req.params.id).populate('companyId').exec()
        .then(job => {

            if (!job) return res.status(404).json({
                error: 'Not Found',
                message: `job not found`
            });

            res.status(200).json(job)
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};

const update = (req, res) => {
    if (Object.keys(req.body).length === 0)
    {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }

    JobModel.findById(req.params.id).exec()
        .then(job => {
            // if (blog.doctorId!=req.userId)
            // {
            //     return res.status(401).send({
            //         error: 'Unauthorized',
            //         message: 'Request not made by the creator.'
            //     });
            // }
            JobModel.findByIdAndUpdate(req.params.id,{title: req.body.title, description: req.body.description, salary: req.body.salary},{
                new: true,
                runValidators: true}).exec()
                .then(job => {
                    if (!job)
                    {
                        return res.status(404).json({
                            error: 'Not Found',
                            message: `job not found`
                        });
                    }
                    res.status(200).json(job)
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

const remove = (req, res) => {
    BlogModel.findById(req.params.id).exec()
        .then((blog) => {
            if (blog.doctorId!=req.userId)
            {
                return res.status(401).send({
                    error: 'Unauthorized',
                    message: 'Request not made by the creator.'
                });
            }
            BlogModel.findById(req.params.id,{comments:true,_id:false}).exec()
                .then(comments => {
                    CommentModel.find({_id: {$in: comments['comments']}}).deleteMany().exec()
                        .then(() => {
                            //All the comments are deleted, Now delete the blog
                            BlogModel.findByIdAndRemove(req.params.id).exec()
                                .then(() => res.status(200).json({message: `Blog with id${req.params.id} was deleted`}))
                                .catch(error => res.status(500).json({
                                    error: 'Internal server error while deleting blog',
                                    message: error.message
                                }));
                        })
                        .catch(error => res.status(500).json({
                            error: 'Internal server error while deleting comments related to deleted blog',
                            message: error.message
                        }));
                })
                .catch(error => res.status(404).json({
                    error: 'Blog with the given Id does not exist',
                    message: error.message
                }));
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};

const listAll  = (req, res) => {
    //return all jobs
    JobModel.find().populate('companyId').exec()
        .then(jobs => res.status(200).json(jobs))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const getFeaturedBlog = (req,res) => {
    BlogModel.find().sort({"updatedAt":-1}).limit(1).populate('doctorId','firstName lastName').
    populate({path:'comments',select:'-blogId',populate: {path:'personId',select:'firstName lastName'}}).exec()
        .then(blog => {
            res.status(200).json(blog[0])
        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));
};



module.exports = {
    create,
    insertComment,
    read,
    update,
    remove,
    listAll,
    getFeaturedBlog
};
