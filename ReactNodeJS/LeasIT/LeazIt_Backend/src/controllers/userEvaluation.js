"use strict";

const UserEvaluationModel = require('../models/userEvaluation')
const UserModel = require('../models/user');

const create = (req, res) => {
    if (Object.keys(req.body).length === 0) { return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });}

    const evaluation = new UserEvaluationModel({
        rating: req.body.rating,
        comment: req.body.comment,
        evaluatee_id: req.params.Evaluatee_Id,
        evaluator_id: req.userId,
        date: Date()

    });
    UserEvaluationModel.create(evaluation)
        .then(evaluation => res.status(201).json(evaluation))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
    }));
};

const readOwnEvaluations = (req, res, next) => {
   /* UserEvaluationModel.findOne({ evaluatee_id: req.userId })
    // .select('_id')
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
*/
    console.log(req.userId)
    UserEvaluationModel.find({ evaluatee_id: req.userId })
    // .select('_id')
        .populate({
            path: 'evaluator_id',
            select: 'username' // TODO -- firstname? username?
        })
        .populate({
            path: 'evaluatee_id',
            select: 'username' // TODO -- firstname? username?
        })
        .exec(function(err, evaluations) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            return res.status(200).json({"evaluations": evaluations});
        });
        };

const readOtherUserEvaluations = (req, res, next) => {
  /*  UserEvaluationModel.find({ evaluatee_id: req.params.UserId })
    // .select('_id')
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
*/

    UserEvaluationModel.find({ evaluatee_id: req.params.UserId })
    // .select('_id')
        .populate({
            path: 'evaluator_id',
            select: 'username' // TODO -- firstname? username?
        })
        .populate({
            path: 'evaluatee_id',
            select: 'username' // TODO -- firstname? username?
        })
        .exec(function(err, evaluations) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            return res.status(200).json({"evaluations": evaluations});
        });
};

const getUserEvaluation = (req, res, next) => {
    UserEvaluationModel.find({_id: req.params.id})
        .populate({
            path: "evaluator_id",
            select : "username"
        })

        .exec(function(err, user_evaluation) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }

            res.status(200).json({ user_evaluation: {user_evaluation: user_evaluation}});
        });
};

const read   = (req, res) => {
    UserEvaluationModel.findById(req.params.id).exec()
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

    UserEvaluationModel.findByIdAndUpdate(req.params.id,req.body,{
        new: true,
        runValidators: true}).exec()
        .then(evaluation => res.status(200).json(evaluation))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

const remove = (req, res) => {
    UserEvaluationModel.findByIdAndRemove(req.params.id).exec()
        .then(() => res.status(200).json({message: `Evaluation with id${req.params.id} was deleted`}))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

const list  = (req, res) => {
    UserEvaluationModel.find({}).exec()
        .then(evaluations => res.status(200).json(evaluations))
.catch(error => res.status(500).json({
        error: 'Internal server error',
        message: error.message
    }));
};

module.exports = {
    create,
    read,
    readOwnEvaluations,
    readOtherUserEvaluations,
    getUserEvaluation,
    update,
    remove,
    list
};