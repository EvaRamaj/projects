"use strict";

const express  = require('express');
const router   = express.Router();
const middlewares    = require('../middlewares');
const FileController = require('../controllers/file');


router.get('/', middlewares.find_all_uploaded_files); // List all movies
router.post('/', middlewares.upload_file.single('file'), (req, res) => {res.json({file : req.file })}); // Create a new movie
router.get('/:filename', middlewares.find_one_uploaded_file); // Read a movie by Id
router.put('/:id', /*middlewares.checkAuthentication,*/  FileController.update); // Update a movie by Id
router.delete('/:id', /*middlewares.checkAuthentication,*/  FileController.remove); // Delete a movie by Id


module.exports = router;