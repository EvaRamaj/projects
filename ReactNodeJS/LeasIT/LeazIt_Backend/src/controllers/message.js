"use strict";
const ObjectId = require('mongodb').ObjectID;
const MessageModel = require('../models/message_old');
const UserModel = require('../models/user');



const create = (req, res) => {



    // use :user_id from url as sender_id
    let sender_id = req.baseUrl.split("/")[2];

    // assign user_id acquired from url to the sender_id field of the body of the request
    req.body.sender_id = sender_id;

    if (Object.keys(req.body).length === 0) {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }
    MessageModel.create(req.body)
        .then(message => {
            res.status(201).json(message);
            res.save(callback)
        })
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));

    let body = {"messages": res._id};
    // console.log(body);

    UserModel.findByIdAndUpdate(sender_id,body,{
        new: true,
        runValidators: true}).exec(callback)
        .then(user => res.status(200).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const read   = (req, res) => {
    // MessageModel.findById(req.params.id).exec()
    //     .then(message => {
    //
    //         if (!message) return res.status(404).json({
    //             error: 'Not Found',
    //             message: `Message not found`
    //         });
    //         // console.log(req.params);
    //         res.status(200).json(message)
    //
    //     })
    //     .catch(error => res.status(500).json({
    //         error: 'Internal Server Error',
    //         message: error.message
    //     }));

    let user_id = req.baseUrl.split("/")[2];
    // console.log(user_id);

    // if the user is sender or resceiver of the requested message show it, otherwise not :D
    MessageModel.findOne({$and: [{"_id" : req.params.id}, {$or: [{"receiver_id" : ObjectId(user_id)}, {"sender_id" : ObjectId(user_id)}]}]}).exec()
        .then(message => {

            if (!message) return res.status(404).json({
                error: 'Not Found',
                message: `Message not found`
            });
            // console.log(req.params);
            res.status(200).json(message)

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

    // MessageModel.findByIdAndUpdate(req.params.id,req.body,{
    //     new: true,
    //     runValidators: true}).exec()
    //     .then(message => res.status(200).json(message))
    //     .catch(error => res.status(500).json({
    //         error: 'Internal server error',
    //         message: error.message
    //     }));

    // use :user_id from url as sender_id
    let sender_id = req.baseUrl.split("/")[2];

    // assign user_id acquired from url to the sender_id field of the body of the request
    req.body.sender_id = sender_id;

    // allows the user to update a message only if they are the authors of the message and not if they are receivers
    MessageModel.findOneAndUpdate({$and: [{"_id" : req.params.id}, {"sender_id" : ObjectId(sender_id)}]},req.body,{
        new: true,
        runValidators: true}).exec()
        .then(message => res.status(200).json(message))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const remove = (req, res) => {
    // MessageModel.findByIdAndRemove(req.params.id).exec()
    //     .then(() => res.status(200).json({message: `Message with id${req.params.id} was deleted`}))
    //     .catch(error => res.status(500).json({
    //         error: 'Internal server error',
    //         message: error.message
    //     }));

    // use :user_id from url as sender_id
    let sender_id = req.baseUrl.split("/")[2];

    // assign user_id acquired from url to the sender_id field of the body of the request
    req.body.sender_id = sender_id;

    MessageModel.findOneAndRemove({$and: [{"_id" : req.params.id}, {"sender_id" : ObjectId(sender_id)}]}).exec()
        .then(() => res.status(200).json({message: `Message with id${req.params.id} was deleted`}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const list  = (req, res) => {

    console.log(req);

    // get users id to use it in the query
    let user_id = req.baseUrl.split("/")[2];

    // find messages of the specific user
    MessageModel.find({$or: [{"receiver_id" : ObjectId(user_id)}, {"sender_id" : ObjectId(user_id)}]}).exec()
        .then(messages => {

            res.status(200).json(messages)
        })
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