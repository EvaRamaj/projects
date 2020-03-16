"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const ItemController = require('../controllers/item');
const BookingController = require('../controllers/booking');
const Role           = require('../role_constants');

// get all items -- can be done by anyone
router.get('/', ItemController.list);
router.get('/my_items',middlewares.checkAuthentication, ItemController.myItems);
// only role Lessor can add a new item
router.post('/', middlewares.checkAuthentication,middlewares.roleAuthorization(Role.ROLE_LESSOR), ItemController.create);
// view item info -- everyone can do that
router.get('/:id', ItemController.read);
// update item's info -- only Lessor can do that
router.put('/:itemId', middlewares.checkAuthentication, ItemController.update);
// delete an item -- only Lessor can do that -- TODO check for current boookings
router.delete('/:id', middlewares.checkAuthentication, ItemController.remove, BookingController.remove);
router.delete('/userItems/:itemId', middlewares.checkAuthentication, ItemController.remove_many, BookingController.remove);
//search partially
router.post('/search',ItemController.searchFunction);


module.exports = router;