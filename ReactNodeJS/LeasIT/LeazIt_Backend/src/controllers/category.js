"use strict";

const CategoryModel = require('../models/category');


const create = (req, res) => {
    if (Object.keys(req.body).length === 0) { return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });}

    CategoryModel.create(req.body)
        .then(user => res.status(201).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const read   = (req, res) => {
    CategoryModel.findById(req.params.id).exec()
        .then(user => {

            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `Movie not found`
            });

            res.status(200).json(user)

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

    CategoryModel.findByIdAndUpdate(req.params.id,req.body,{
        new: true,
        runValidators: true}).exec()
        .then(user => res.status(200).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const remove = (req, res) => {
    CategoryModel.findByIdAndRemove(req.params.id).exec()
        .then(() => res.status(200).json({message: `Movie with id${req.params.id} was deleted`}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const list  = (req, res) => {
    CategoryModel.find({}).exec()
        .then(users => res.status(200).json(users))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};



module.exports = {
    create,
    read,
    update,
    remove,
    list
};