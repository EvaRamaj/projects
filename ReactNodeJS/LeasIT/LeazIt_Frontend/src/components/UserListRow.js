"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';


export class UserListRow extends React.Component {

    constructor(props) {
        console.log(props)
        super(props);
    }

    render() {
        if (this.props.lessor_req) {
            return (
                <TableRow key={this.props.key}>
                    <TableColumn><Link
                        to={`/profile/${this.props.user._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
                    <TableColumn><SimpleLink
                        to={`/profile/${this.props.user._id}`}>{this.props.user.username}</SimpleLink></TableColumn>
                    {AuthService.isAuthenticated() ?
                        <TableColumn><Button onClick={() => this.props.onDelete(this.props.user._id)}
                                             icon>delete</Button></TableColumn>
                        : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                    }
                    {AuthService.isAuthenticated() ?
                        <TableColumn><SimpleLink
                            to={`/id_doc/${this.props.user._id}`}>Examine Request</SimpleLink></TableColumn>
                        : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                    }

                </TableRow>
            );
        }
        else {
            return (
                <TableRow key={this.props.key}>
                    <TableColumn><Link
                        to={`/profile/${this.props.user._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
                    <TableColumn><SimpleLink
                        to={`/profile/${this.props.user._id}`}>{this.props.user.username}</SimpleLink></TableColumn>
                    {AuthService.isAuthenticated() ?
                        <TableColumn><Button onClick={() => this.props.onDelete(this.props.user._id)}
                                             icon>delete</Button></TableColumn>
                        : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                    }

                </TableRow>
            );
        }
    }
}