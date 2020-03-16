"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';


export class UserEvaluationListRow extends React.Component {

    constructor(props) {
        super(props);
        console.log(this.props)
    }

    render() {
        let user_evaluation  = this.props.user_evaluation;
        return (
            <TableRow key={this.props.key}>
                <TableColumn><Link
                    to={`/user_evaluation/${this.props.user_evaluation._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
                <TableColumn><SimpleLink
                    to={`/user_evaluation/${this.props.user_evaluation._id}`}>{this.props.user_evaluation._id}</SimpleLink></TableColumn>
                {AuthService.isAuthenticated() ?
                    <TableColumn><Link
                        to={`/user_evaluations/${this.props.user_evaluation._id}`}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                }
                {AuthService.isAuthenticated() ?
                    <TableColumn><Button onClick={() => this.props.onDelete(this.props.user_evaluation._id)}
                                         icon>delete</Button></TableColumn>
                    : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                }


            </TableRow>
        );
    }
}
