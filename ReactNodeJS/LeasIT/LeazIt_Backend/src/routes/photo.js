"use strict";

const express  = require('express');
const router   = express.Router();
const middlewares    = require('../middlewares');
const PhotoController = require('../controllers/photo');


router.get('/', middlewares.find_all_uploaded_photos); // List all movies
router.post('/', middlewares.upload_photo.single('photo'), (req, res) => {res.json({file : req.file })}); // Create a new movie
router.post('/many', middlewares.upload_photo.array('photo',5), (req, res) => {res.json({file : req.file })}); // Create a new movie
router.get('/:filename',/*middlewares.checkAuthentication,*/ middlewares.find_one_uploaded_photo); // Read a movie by Id
router.put('/:id', /*middlewares.checkAuthentication,*/ PhotoController.update); // Update a movie by Id
router.delete('/:id', /*middlewares.checkAuthentication,*/ PhotoController.remove); // Delete a movie by Id


module.exports = router;