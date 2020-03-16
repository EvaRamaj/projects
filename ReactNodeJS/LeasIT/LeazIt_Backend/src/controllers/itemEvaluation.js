"use strict";

const ItemEvaluationModel = require('../models/itemEvaluation')
const BookingModel = require('../models/booking')


const create = (req, res) => {
    if (Object.keys(req.body).length === 0) { return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });}



    const evaluation = new ItemEvaluationModel({
        item_id: req.params.ItemId,
        evaluator_id: req.userId,

    });
    //check again
    BookingModel.find({ item_id: req.params.itemId, _id: req.params.id })
        .exec(function (err, booking) { if (err){res.status(404).send({error: err}); return next(err)}});
    ItemEvaluationModel.create(evaluation)
        .then(evaluation => res.status(201).json(evaluation))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
    }));
};

const readAnyItemEvaluations= (req, res) => {
    ItemEvaluationModel.find({ item_id: req.params.ItemId })
    // .select('_id')
        .exec()
        .then(evaluations => {

            if (!evaluations) return res.status(404).json({
                error: 'Not Found',
                message: `Evaluation not found`
            });

            res.status(200).json(evaluations)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};

const read   = (req, res) => {
    ItemEvaluationModel.findById(req.params.id)
        .populate({
            path: 'evaluator_id',
            select: 'username'
        })
        .exec()
        .then(evaluation => {

            if (!evaluation) return res.status(404).json({
            error: 'Not Found',
             message: `Evaluation not found`
            });

             res.status(200).json(evaluation)

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

    ItemEvaluationModel.findByIdAndUpdate(req.params.id,req.body,{
        new: true,
        runValidators: true}).exec()
        .then(evaluation => res.status(200).json(evaluation))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

const remove = (req, res) => {
    ItemEvaluationModel.findByIdAndRemove(req.params.id).exec()
        .then(() => res.status(200).json({message: `Evaluation with id${req.params.id} was deleted`}))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

const list  = (req, res) => {
    ItemEvaluationModel.find({}).exec()
        .then(evaluations => res.status(200).json({"evaluations": evaluations}))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

const getItemEvaluation = (req, res, next) => {
    ItemEvaluationModel.find({_id: req.params.id})
        .populate({
            path: "evaluator_id",
            select : "username"
        })
        .populate({
            path: "item_id",
            select : "photos"
        })

        .exec(function(err, item_evaluation) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }

            res.status(200).json({ item_evaluation: {item_evaluation: item_evaluation}});
        });
};

module.exports = {
    create,
    read,
    readAnyItemEvaluations,
    update,
    remove,
    list,
    getItemEvaluation
};