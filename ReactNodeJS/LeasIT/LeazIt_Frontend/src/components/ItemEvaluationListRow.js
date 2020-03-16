"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';

export class ItemEvaluationListRow extends React.Component {

    constructor(props) {
        super(props);
        console.log("alex")
        console.log(props)
        console.log("alex")
    }

    render() {
        console.log("edw")
        console.log(this.props.item_evaluation._id)
         let item_evaluation  = this.props.item_evaluation._id;
        console.log(item_evaluation)
        console.log("ekei")
        return (

            <TableRow key={this.props.key}>
                <TableColumn><Link
                    to={`/profile/${item_evaluation}`}><FontIcon>image</FontIcon></Link></TableColumn>
                <TableColumn><SimpleLink
                    to={`/profile/${item_evaluation}`}>{item_evaluation}</SimpleLink></TableColumn>
                {AuthService.isAuthenticated() ?
                    <TableColumn><Link
                        to={`/edit/${item_evaluation}`}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                }
                {AuthService.isAuthenticated() ?
                    <TableColumn><Button onClick={() => this.props.onDelete(item_evaluation._id)}
                                         icon>delete</Button></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                }

            </TableRow>
        );

    }
}