"use strict";


const multer        = require('multer');
const mongoose      = require('mongoose');
const path          = require('path');
const crypto        = require('crypto');
const GridFsStorage = require('multer-gridfs-storage');
const Grid          = require('gridfs-stream');

const jwt           = require('jsonwebtoken');

const config        = require ('./config');
const Helper        = require ('./helpers');
const UserModel     = require('./models/user');

const allowCrossDomain = (req, res, next) => {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS');
    res.header('Access-Control-Allow-Headers', '*');

    // intercept OPTIONS method
    if ('OPTIONS' == req.method) {
        res.status(200).send(200);
    }
    else {
        next();
    }
};

const checkAuthentication = (req, res, next) => {

    // check header or url parameters or post parameters for token
    const token = req.headers['x-access-token'];

    if (!token)
        return res.status(401).send({
            error: 'Unauthorized',
            message: 'No token provided in the request'
        });

    // verifies secret and checks exp
    jwt.verify(token, config.JwtSecret, (err, decoded) => {
        if (err) return res.status(401).send({
            error: 'Unauthorized',
            message: 'Failed to authenticate token.'
        });

        // if everything is good, save to request for use in other routes
        req.userId = decoded.id;
        next();
    });


};

const errorHandler = (err, req, res, next) => {
    if (res.headersSent) {
        return next(err)
    }
    res.status(500);
    res.render('error', { error: err })
};

const conn = mongoose.createConnection(config.mongoURI);
let gfs_photos;
let gfs_files;

conn.once('open', function () {
    gfs_photos = Grid(conn.db, mongoose.mongo);
    gfs_photos.collection('photos');
    gfs_files = Grid(conn.db, mongoose.mongo);
    gfs_files.collection('files');
    // all set!
});

var storage_photos = new GridFsStorage({
    url: config.mongoURI,
    file: (req, file) => {
        console.log(req);
        return new Promise((resolve, reject) => {
            crypto.randomBytes(16, (err, buf) => {
                if (err) {
                    return reject(err);
                }
                const filename = buf.toString('hex') + path.extname(file.originalname);
                console.log(req.userId);
                const fileInfo = {
                    filename: filename,
                    owner_id: req.userId,
                    bucketName: 'photos'
                };
                resolve(fileInfo);
            });
        });
    }
});
const upload_photo = multer({ storage: storage_photos });
const upload_photos = multer({ storage: storage_photos });
const find_all_uploaded_photos = (req, res) => {gfs_photos.files.find().toArray((err, files) => {return res.json(files)});};
const find_one_uploaded_photo = (req, res) => {
    gfs_photos.files.findOne({filename : req.params.filename}, (err, file) => {
        if (err) {
            return err;
        }
        const readstream = gfs_photos.createReadStream(file.filename);
        readstream.pipe(res)
        ;});};

var storage_files = new GridFsStorage({
    url: config.mongoURI,
    file: (req, file) => {
        return new Promise((resolve, reject) => {
            crypto.randomBytes(16, (err, buf) => {
                if (err) {
                    return reject(err);
                }
                const filename = buf.toString('hex') + path.extname(file.originalname);
                console.log(req.userId);
                const fileInfo = {
                    filename: filename,
                    owner_id : req.userId,
                    bucketName: 'files'
                };
                resolve(fileInfo);
            });
        });
    }
});

// Role authorization check
const roleAuthorization = (requiredRole) => {
    return function (req, res, next) {
        // const user = req.user;

        UserModel.findById(req.userId, (err, foundUser) => {
            if (err) {
                res.status(422).json({ error: 'No user was found.' });
                return next(err);
            }

            // If user is found, check role.
            if (Helper.getRole(foundUser.role) >= Helper.getRole(requiredRole)) {
                return next();
            }

            return res.status(401).json({ error: 'You are not authorized to view this content.' });
        });
    };
};


const upload_file = multer({ storage: storage_files });
const find_all_uploaded_files = (req, res) => {gfs_photos.files.find().toArray((err, files) => {return res.json(files)});};
const find_one_uploaded_file = (req, res) => {
    gfs_files.files.findOne({filename : req.params.filename}, (err, file) => {
        if (err) {
            return err;
        }
        const readstream = gfs_files.createReadStream(file.filename); readstream.pipe(res);
    });};



module.exports = {
    allowCrossDomain,
    checkAuthentication,
    roleAuthorization,
    upload_photo,
    upload_photos,
    find_all_uploaded_photos,
    find_one_uploaded_photo,
    upload_file,
    find_all_uploaded_files,
    find_one_uploaded_file,
    errorHandler

};