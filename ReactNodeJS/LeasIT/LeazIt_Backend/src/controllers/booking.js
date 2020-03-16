"use strict";

const BookingModel = require('../models/booking');
const ItemModel = require('../models/item');


const create = (req, res) => {
    if (Object.keys(req.body).length === 0) { return res.status(400).json({
        error: 'Bad Request',
        message: 'The request body is empty'
    });}

    BookingModel.create(req.body)
        .then(user => res.status(201).json(user))
        .catch(error => res.status(500).json({
            error: 'Internal server error',
            message: error.message
        }));
};

const read   = (req, res) => {
    BookingModel.findById(req.params.id).exec()
        .then(user => {

            if (!user) return res.status(404).json({
                error: 'Not Found',
                message: `Booking not found`
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
    const booking = new BookingModel({
        item_id: req.params.itemId,
        booker_id : req.userId,
        //bookingDate: newDate(),
        //startDate : req.body.startDate,
        //endDate: req.body.endDate,
    });
    BookingModel.findOneAndUpdate({
            $and : [
                { '_id': req.params.id }, { 'booker_id': req.userId }
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

const remove = (req, res) => {
    BookingModel.deleteMany({ item_id: req.params.itemId}, function (err) {
        console.log(req.params.itemId);
        if (err) {
            res.send({error: err});
            return next(err);
        }
        return next()
    });
};

const getMyBookings = (req, res, next) => {
    BookingModel.find({ booker_id: req.userId }) //booker: enoikiastis
    // .select('_id')
        .exec(function(err, bookingItems) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            return res.status(200).json({"bookingItems": bookingItems});
        });
};

const getItemBooking = (req, res, next) => {
    ItemModel.find({'_id': req.params.itemId ,'owner_id': req.userId
    }).exec(function (err, item) { if (err){res.status(404).send({error: err}); return next(err)}});
    BookingModel.find({ item_id: req.params.itemId, _id: req.params.id }) //booker: enoikiastis
    // .select('_id')
        .populate(
            {
                path: "booker_id",
                select: "username photo"
            }
        )
        .populate({
                path: "item_id",
                select:"name price photos"
            }
        )
        .exec(function(err, bookingItems) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            console.log('here:', bookingItems);
            return res.status(200).json({"bookingItems": bookingItems});
        });
};
const getItemBookings = (req, res, next) => {
    ItemModel.find({'_id': req.params.itemId ,'owner_id': req.userId
        }).exec(function (err, item) { if (err){res.status(404).send({error: err}); return next(err)}});
      BookingModel.find({ item_id: req.params.itemId }) //booker: enoikiastis
      // .select('_id')
        .populate(
            {
                path: "booker_id",
                select: "username photo"
            }
        )
          .populate({
                  path: "item_id",
                  select:"name photo"
            }
          )
        .exec(function(err, bookingItems) {
            if (err) {
                res.send({error: err});
                return next(err);
            }
            // console.log(conversations);
            // Set up empty array to hold conversations + most recent message
            return res.status(200).json({"bookingItems": bookingItems});
        });
};

const getBooking = (req, res, next) => {
    BookingModel.find({_id: req.params.id, booker_id: req.userId })
        .populate(
            {
                path: "booker_id",
                select: "username photo"
            }
        )
        .populate(
            {
                path: "item_id",
                select: "name photos owner_id"
            }
        )
        .exec(function(err, booking) {
            if (err) {
                res.send({ error: err });
                return next(err);
            }
            console.log('getBooking:' ,booking);
            res.status(200).json({ booking: {booking: booking}});
        });
};

const newBooking = (req, res, next) =>{

    if(!req.params.itemId) {
        res.status(422).send({ error: 'Please choose a valid item .' });
        return next();
    }
    const booking = new BookingModel({
        item_id: req.params.itemId,
        booker_id : req.userId,
        bookingDate: Date(),
        startDate : req.body.startDate,
        endDate: req.body.endDate,
    });
    BookingModel.create(booking,function(err, booking) {
        if (err) {
            console.log(err);
            res.send({ error: err });
            return next(err);
        }
            res.status(200).json({ booking: booking });
            return next();
        });
};


const list  = (req, res) => {
    BookingModel.find({}).exec()
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
    getMyBookings,
    getItemBookings,
    getItemBooking,
    getBooking,
    newBooking,
    list};