"use strict";

const UserModel = require('../models/user');
const ItemModel = require('../models/item');
const BookingModel = require('../models/booking');



const create = (req, res) => {
    if (Object.keys(req.body).length === 0) return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });

    UserModel.create(req.body)
        .then(user => res.status(201).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const getUserByUsername   = (req, res) => {
    UserModel.findOne({$and: [{$or : [{'role':'Lessee'}, {'role':'Lessor'}]}, {'username': req.params.username}]})
        .exec()
        .then(user => {
            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `User not found`
            });
            res.status(200).json(user._id)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};
const view_user_profile   = (req, res) => {
    UserModel.findOne({$and: [{$or : [{'role':'Lessee'}, {'role':'Lessor'}]}, {'_id': req.params.id}]})
        .exec()
        .then(user => {
            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `User not found`
            });
            console.log(user);
            user.password='';
            res.status(200).json(user)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};

const get_profile   = (req, res) => {
    UserModel.findById(req.userId).exec()
        .then(user => {

            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `User not found`
            });

            res.status(200).json(user)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};

const update_profile = (req, res) => {
    if (Object.keys(req.body).length === 0)
    {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }
    UserModel.findByIdAndUpdate(req.userId,req.body,{
        new: true,
        runValidators: true}).exec()
        .then(user => res.status(200).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const remove_user = (req, res) => {
    UserModel.findByIdAndRemove(req.params.id).exec()
        .then(() => res.status(200).json({message: `User with id${req.params.id} was deleted`}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const delete_profile = (req, res) => {

    // --TODO find booking dates and if we have overlapings do not allow delete profile
    let dateTime = require('node-datetime');
    let dt = dateTime.create();
    let formatted = dt.format('Y-m-d H:M:S');

    if(dt._now > Date.parse('2018-06-17T12:22:43.451Z')){
        console.log(dt._now);
    }

    UserModel.findByIdAndRemove(req.userId).exec()
        .then(() => res.status(200).json({message: `User with id${req.userId} was deleted`}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));

    // UserModel.findByIdAndRemove(req.params.id).exec()
    //     .then(() => res.status(200).json({message: `User with id${req.params.id} was deleted`}))
    //     .catch(error => res.status(500).json({
    //         error: 'Internal server error',
    //         message: error.message
    //     }));
};

const view_all_users  = (req, res) => {
    UserModel.find({}).exec()
        .then(users => res.status(200).json(users))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const upgrade_request = (req, res) => {

    UserModel.findOneAndUpdate({'_id' :req.userId}, {$set :{lessor_role_requested: req.body.request}}).exec()
        .then(()=>res.status(200).json({message: 'You successfully requested to become a lessor!'}))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const grant_lessor_privileges = (req, res) => {
    UserModel.findOneAndUpdate({$and: [{'_id' :req.params.id}, {lessor_role_requested: true}]}, {$set :{is_Lessor: req.body.is_Lessor, role: req.body.role, lessor_role_requested: req.body.lessor_role_requested}}).exec()
        .then(user=>{

            if (!user) return res.status(404).json({
                error: 'Not fount',
                message: `User did not request to become a lessor!`
            });

            let message_js;
            if(req.body.is_Lessor) {
                message_js= {message: `Successfully upgraded user with id ${req.params.id} to lessor!`};
                return res.status(200).json(message_js);
            }
            else{
                message_js = {message: `User with id ${req.params.id} is not upgraded to lessor!`};
                return res.status(200).json(message_js);
            }

            // if(!req.body.decision)
            //     return res.status(200).json({message: `User with id ${req.params.id} is not upgraded to lessor!`});

        })
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const view_all_lessor_request  = (req, res) => {
    UserModel.find({lessor_role_requested: true}).exec()
        .then(users => res.status(200).json(users))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};
// do we have to view each profile of lessee to update -- TODO



module.exports = {
    create,
    view_user_profile,
    getUserByUsername,
    update_profile,
    remove_user,
    view_all_users,
    get_profile,
    upgrade_request,
    grant_lessor_privileges,
    view_all_lessor_request,
    delete_profile
};