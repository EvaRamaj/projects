"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';
import minion from '../images/minion.jpg';



export class MessageListRow extends React.Component {

    constructor(props) {
        super(props);
        var str = this.props.message.createdAt.split('T');
        var temp = str[0].split('-');
        var date = temp[2].toString() + '-' + temp[1].toString() + '-' + temp[0].toString();
        str[1] = str[1].split('.');
        var time =str[1][0];
        var date_time = time.toString() + " " + date.toString();
        this.props.message.createdAt = date_time;
        if(this.props.message.author.username === this.props.username){
            this.state={
                loading: false,
                author_is_current_user: true
            }
        }
        else{
            this.state={
                loading: false,
                author_is_current_user: false
            };
        }
    }

    render() {
        if(this.state.loading){
            return(
                <h2>Loading....</h2>
            )
        }
        if(this.state.author_is_current_user === true)
        {
            return (
                <div className="message-candidate center-block">
                    <div className="row">
                        <div className="col-xs-8 col-md-6">
                            <img src= {`http://localhost:3000/photos/${this.props.user.photo}`}  className="message-photo" />
                            <h4 className="message-name">{this.props.message.author.username}</h4>
                        </div>
                        <div className="col-xs-4 col-md-6 text-right message-date">
                            {this.props.message.createdAt.toString()}</div>
                    </div>
                    <div className="row message-text">
                        {this.props.message.body}
                    </div>
                </div>
            );
        }
        else{
            return(
                <div className="message-hiring-manager center-block">
                    <div className="row">
                        <div className="col-xs-8 col-md-6">
                            <img src= {`http://localhost:3000/photos/${this.props.other_user.photo}`} className="message-photo" />
                            <h4 className="message-name">{this.props.message.author.username}</h4>
                        </div>
                        <div className="col-xs-4 col-md-6 text-right message-date">{this.props.message.createdAt.toString()}</div>
                    </div>
                    <div className="row message-text ">
                        {this.props.message.body}
                    </div>
                </div>
            );
        }
    }
            // <TableRow key={this.props.key}>
            //     <TableColumn>{this.props.message.author.username}</TableColumn>
            //     <TableColumn>{this.props.message.recipient.username}</TableColumn>
            //     <TableColumn>{this.props.message.body}</TableColumn>
            // </TableRow
}