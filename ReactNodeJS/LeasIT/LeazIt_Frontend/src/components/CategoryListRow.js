"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';


export class CategoryListRow extends React.Component {

    constructor(props) {
        console.log(props)
        super(props);
    }

    render() {
        return (
            <TableRow key={this.props.key}>
                <TableColumn><Link to={`/item/${this.props.categories._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
                <TableColumn><SimpleLink to={`/item/${this.props.categories._id}`}>{this.props.categories.name}</SimpleLink></TableColumn>
                {AuthService.isAuthenticated() ?
                    <TableColumn><Link to={`/edit/${this.props.categories._id}`}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                }
                {AuthService.isAuthenticated() ?
                    <TableColumn><Button onClick={() => this.props.onDelete(this.props.categories._id)} icon>delete</Button></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                }

            </TableRow>
        );
    }
}