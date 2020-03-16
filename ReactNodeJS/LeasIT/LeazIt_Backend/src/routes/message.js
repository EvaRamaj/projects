"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const MessageController = require('../controllers/message');

// code without checking auhtentication

router.get('/', MessageController.list); // List all movies
router.post('/', MessageController.create); // Create a new movie
router.get('/:id', MessageController.read); // Read a movie by Id
router.put('/:id', MessageController.update); // Update a movie by Id
router.delete('/:id', MessageController.remove); // Delete a movie by Id

// usefull code for checking authentication too

// router.post('/', middlewares.checkAuthentication, MessageController.create); // Create a new movie
// router.get('/:id', MessageController.read); // Read a movie by Id
// router.put('/:id', middlewares.checkAuthentication, MessageController.update); // Update a movie by Id
// router.delete('/:id', middlewares.checkAuthentication, MessageController.remove); // Delete a movie by Id


module.exports = router;