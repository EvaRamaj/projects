"use strict";

const ConversationModel = require('../models/conversation');
const MessageModel = require('../models/message');
// const UserModel = require('../models/user');

const getConversations = (req, res,next) => {
// exports.getConversations = function(req, res, next) {

    // console.log(req.userId);

    // ConversationModel.find({}).exec()
    //     .then(conversations => {
    //
    //         res.status(200).json(conversations)
    //     })
    //     .catch(error => res.status(500).json({
    //         error: 'Internal server error',
    //         message: error.message
    //     }));



    // Only return one message from each conversation to display as snippet
    ConversationModel.find({ participants: req.userId })
        // .select('_id')
        .exec(function(err, conversations) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            let fullConversations = [];
            conversations.forEach(function(conversation) {
                MessageModel.find({ 'conversationId': conversation._id })
                    .sort({'createdAt': -1})    //sort does not work correctly in postman but works on db
                    .limit(1)
                    .populate({
                        path: "author",
                        select: 'username password' // TODO -- firstname? username?
                    })
                    .exec(function(err, message) {
                        if (err) {
                            res.send({ error: err });
                            return next(err);
                        }
                        // console.log(conversation)
                        fullConversations.push({'conversationId' : conversation._id, 'participants' : conversation.participants, 'messages' : message});
                        if(fullConversations.length === conversations.length) {
                            return res.status(200).json({ conversations:  fullConversations });
                        }
                    });
            });
        });
};

const getConversation = (req, res, next) => {
    MessageModel.find({conversationId: req.params.conversationId })
        // .select('createdAt body author') //body author
        .sort('-createdAt')
        .populate({
            path: 'author',
            select: 'username password' // TODO -- firstname? username?
        })
        .exec(function(err, messages) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }

            res.status(200).json({ conversation: {'messages' : messages}});
        });
};

const newConversation = (req, res, next) =>{

    if(!req.params.recipient) {
        res.status(422).send({ error: 'Please choose a valid recipient for your message.' });
        return next();
    }

    if(!req.body.composedMessage) {
        res.status(422).send({ error: 'Please enter a message.' });
        return next();
    }
    // console.log(req.userId, req.params.recipient);

    const conversation = new ConversationModel({
        participants: [req.userId, req.params.recipient]
    });

    // console.log(conversation);


    // ConversationModel.create({participants: [req.userId, req.params.recipient]})
    //     .then(conversation => {
    //         res.status(201).json(conversation);
    //     })
    //     .catch(error => res.status(500).json({
    //         error: 'Internal server error',
    //         message: error.message
    //     }));

    ConversationModel.create(conversation,function(err, conversation) {
        if (err) {
            console.log(err);
            res.send({ error: err });
            return next(err);
        }
        // console.log(conversation);

        // console.log(req.userId);
        // res.status(200).json({ message: 'Conversation started!', conversationId: conversation });
        const message = new MessageModel({
            conversationId: conversation._id,
            body: req.body.composedMessage,
            author: req.userId
        });
        // console.log(message);

        MessageModel.create(message,function(err, message) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }

            res.status(200).json({ message: 'Conversation started!', conversation: conversation });
            return next();
        });
    });
};

const sendReply = (req, res, next) =>{
    const reply = new MessageModel({
        conversationId: req.params.conversationId,
        body: req.body.composedMessage,
        author: req.userId
    });

    reply.save(function(err, sentReply) {
        if (err) {
            res.send({ error: err });
            return next(err);
        }

        res.status(200).json({ message: 'Reply successfully sent!' });
        return(next);
    });
};
//
// DELETE Route to Delete Conversation
const deleteConversation = (req, res, next) => {
    ConversationModel.findOneAndRemove({
        $and : [
            { '_id': req.params.conversationId }, { 'participants': req.userId }
        ]}, function(err) {
        if (err) {
            res.send({ error: err });
            return next(err);
        }

        res.status(200).json({ message: 'Conversation removed!' });
        return next();
    });
};

// DELETE method deletes a message if the user is the author
const deleteMessage = (req, res, next) => {
    MessageModel.findOneAndRemove({
        $and : [
            { '_id': req.params.messageId }, { 'author': req.userId }, {'conversationId': req.params.conversationId}
        ]}, function(err) {
        if (err) {
            res.send({ error: err });
            return next(err);
        }

        res.status(200).json({ message: 'Message deleted!' });
        return next();
    });
};

// PUT Route to Update Message
const updateMessage = (req, res, next) => {
    MessageModel.findOneAndUpdate({
        $and : [
            { '_id': req.params.messageId }, { 'author': req.userId }, {'conversationId': req.params.conversationId}
        ]},
        req.body,{
        new: true,
        runValidators: true}).exec()
        .then(message => {
            const resp = {'server response': 'Message successfully updated', 'updated message' :  message};
            res.status(200).json(resp)
        })
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};


module.exports = {
    getConversations,
    newConversation,
    sendReply,
    getConversation,
    updateMessage,
    deleteConversation,
    deleteMessage
};

