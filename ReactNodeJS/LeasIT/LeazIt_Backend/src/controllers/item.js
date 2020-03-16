"use strict";

const ItemModel = require('../models/item');



const create = (req, res,next) => {
    if (Object.keys(req.body).length === 0) { return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });}

    if(!req.body.name) {
        res.status(422).send({ error: 'Please add a name for your Item.' });
        return next();
    }
    if(!req.body.condition) {
        res.status(422).send({ error: 'Please add your Item\'s condition.' });
        return next();
    }
    if(!req.body.categories) {
        res.status(422).send({ error: 'Please add a category for your Item.' });
        return next();
    }
    if(!req.body.description) {
        res.status(422).send({ error: 'Please add a description for your Item.' });
        return next();
    }
    if(!req.body.price) {
        res.status(422).send({ error: 'Please add a price for your Item.' });
        return next();
    }
    /*if(!req.params.pick_up_address) {
        res.status(422).send({ error: 'Please add a pick-up address for your Item.' });
        return next();
    }*/
    /*if(!req.body.availability) {
        res.status(422).send({ error: 'Please add availability for your Item.' });
        return next();
    }*/

    if(!req.body.photos) {
        res.status(422).send({ error: 'Please add photos for your Item.' });
        return next();
    }

    const item = new ItemModel({
        owner_id : req.userId,
        name: req.body.name,
        condition: req.body.condition,
        categories: req.body.categories,
        description: req.body.description,
        price: req.body.price,
        //pick_up_address: req.body.pick_up_address,
        //availability: req.body.availability,
        photos: req.body.photos
    });

    ItemModel.create(item,function(err, new_item) {
        if (err) {
            res.send({ error: err });
            return next(err);
        }

        res.status(200).json({ message: 'Item created', item: item });
        return next();
        });
};

const read   = (req, res) => {
    ItemModel.findById(req.params.id)
        .populate({
                path: 'owner_id',
                select: 'username' // TODO -- firstname? username?
            })
        .populate({
            path: 'categories',
            select: 'name' // TODO -- firstname? username?
        })
        .exec()
        .then(item => {

            if (!item) return res.status(404).json({
                error: 'Not Found',
                message: `Movie not found`
            });

            res.status(200).json(item)

        })
        .catch(error => res.status(500).json({
            error: 'Internal Server Error',
            message: error.message
        }));

};

const update = (req, res, next) => {
    if (Object.keys(req.body).length === 0)
    {
        return res.status(400).json({
            error: 'Bad Request',
            message: 'The request body is empty'
        });
    }

    if(!req.body.name) {
        res.status(422).send({ error: 'Please add a name for your Item.' });
        return next();
    }
    if(!req.body.condition) {
        res.status(422).send({ error: 'Please add your Item\'s condition.' });
        return next();
    }
    if(!req.body.categories) {
        res.status(422).send({ error: 'Please add a category for your Item.' });
        return next();
    }
    if(!req.body.description) {
        res.status(422).send({ error: 'Please add a description for your Item.' });
        return next();
    }
    if(!req.body.price) {
        res.status(422).send({ error: 'Please add a price for your Item.' });
        return next();
    }
    if(!req.params.pick_up_address) {
        res.status(422).send({ error: 'Please add a pick-up address for your Item.' });
        return next();
    }
    if(!req.body.availability) {
        res.status(422).send({ error: 'Please add availability for your Item.' });
        return next();
    }
    if(!req.body.photos) {
        res.status(422).send({ error: 'Please add photos for your Item.' });
        return next();
    }
    const Item = new ItemModel({
        _id : req.params.itemId,
        owner_id : req.userId,
        name: req.body.name,
        condition: req.body.condition,
        categories: req.body.categories,
        description: req.body.description,
        price: req.body.price,
        pick_up_address: req.body.pick_up_address,
        availability: req.body.availability,
        photos: req.body.photos
    });

    ItemModel.findOneAndUpdate({
            $and : [
                { '_id': req.params.itemId }, { 'owner_id': req.userId }
            ]},
        Item,{
            new: true,
            runValidators: true}).exec()
        .then(item => {
            const resp = {'server response': 'Item successfully updated', 'updated item' :  item};
            res.status(200).json(resp)
        })
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const remove = (req, res) => {
    ItemModel.findOneAndRemove({
        $and : [
            { '_id': req.params.itemId }, { 'owner_id': req.userId }
        ]}, function(err) {
        if (err) {
            res.send({ error: err });
            return next(err);
        }

        res.status(200).json({ message: 'Item removed!' });
        return next();
    });
};

const remove_many = (req, res, next) => {
    var query = ItemModel.find({owner_id: req.userId });
    query.select('_id');
    var promise = query.exec(function(err, itemId) {
        if (err) {
            res.send({error: err});
            return next(err);
        }
    });
    var promise = query.exec();
    let itemId_list = [];
    promise.then(function (doc) {
        for (var i = 0; i < doc.length; i++) {
            itemId_list.push(doc[i]._id.toString());
        }
        ItemModel.deleteMany({ _id: { $in : itemId_list }}, function (err) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            return next();
        });
    });
};

const myItems  = (req, res) => {
    ItemModel.find({owner_id: req.userId}).exec()
        .then(items => res.status(200).json(items))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const list  = (req, res) => {
    ItemModel.find({}).exec()
        .then(items => res.status(200).json(items))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const searchFunction  = (req, res) => {
    var q = req.body.q;
    var regex = new RegExp(q);
    console.log(q);
    console.log("this is regex",regex);
    ItemModel.find({
        $or:
            [
                {"name" : { $regex: regex, $options: 'i' }},
                {"description" : { $regex: regex, $options: 'i'} }
                ]})
        .exec().then(items => res.status(200).json(items))
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
    remove_many,
    myItems,
    list,
    searchFunction
};