"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';


export class ItemListRow extends React.Component {

    constructor(props) {
        console.log(props)
        super(props);
    }

    render() {
        return (
            <div className="row">
                <div className="col-2">
                    <Link to={`/item/${this.props.items._id}`}><FontIcon>image</FontIcon></Link>
                </div>
                <div className="col-2">
                    <SimpleLink to={`/item/${this.props.items._id}`}>{this.props.items.name}</SimpleLink>
                </div>
            </div>
        );
    }
}