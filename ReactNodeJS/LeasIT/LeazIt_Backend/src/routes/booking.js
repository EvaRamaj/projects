"use strict";

const express  = require('express');
const router   = express.Router();
const middlewares    = require('../middlewares');
const BookingController = require('../controllers/booking');


// get all bookings ot the authenticated user
router.get('/', middlewares.checkAuthentication, BookingController.getMyBookings);
// get booking depending on itemId -- i.e. user views all bookings of each item -- only lessor can do that
router.get('/details/:id', middlewares.checkAuthentication, BookingController.getBooking);
router.get('/:itemId', middlewares.checkAuthentication, BookingController.getItemBookings);
router.get('/:itemId/details/:id', middlewares.checkAuthentication, BookingController.getItemBooking);
// gets an itemId and books the item -- user should be logged in
router.post('/:itemId', middlewares.checkAuthentication, BookingController.newBooking);

// --TODO which dates the idem is available can be viewed by everyone get_feature bookings

// cancel a booking --TODO owner should contact admin but lessee can cancel a booking -- can be performed if current date < booking date
router.put('/:id', BookingController.update);

// --TODO delete in order to be used in items controller -- deletes all bookings of an item


module.exports = router;